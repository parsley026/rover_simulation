# Copyright (c) 2026. All rights reserved.
# ROSA AI Agent Node — Real-time Asynchronous Tool Telemetry and Non-blocking Execution.

import asyncio
import json
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from rosa import ROSA, RobotSystemPrompts


def build_llm(
    backend: str,
    model: str,
    base_url: str,
    temperature: float,
    api_key: str = 'sk-no-key-required',
):
    """Factory function to instantiate the correct LangChain LLM from config."""
    if backend == 'ollama':
        try:
            from langchain_ollama import ChatOllama
        except ImportError:
            raise ImportError('langchain-ollama is not installed. Run: pip install langchain-ollama')
        return ChatOllama(model=model, base_url=base_url, temperature=temperature)

    elif backend in ('openai', 'vllm'):
        try:
            from langchain_openai import ChatOpenAI
        except ImportError:
            raise ImportError('langchain-openai is not installed. Run: pip install langchain-openai')
        kwargs = dict(model=model, temperature=temperature, api_key=api_key)
        if base_url:
            kwargs['base_url'] = base_url
        return ChatOpenAI(**kwargs)

    elif backend == 'azure':
        try:
            from langchain_openai import AzureChatOpenAI
        except ImportError:
            raise ImportError('langchain-openai is not installed. Run: pip install langchain-openai')
        return AzureChatOpenAI(
            azure_endpoint=base_url,
            model=model,
            temperature=temperature,
            api_key=api_key,
        )

    elif backend == 'anthropic':
        try:
            from langchain_anthropic import ChatAnthropic
        except ImportError:
            raise ImportError('langchain-anthropic is not installed. Run: pip install langchain-anthropic')
        return ChatAnthropic(model=model, temperature=temperature, api_key=api_key)

    else:
        raise ValueError(f"Unknown llm_backend '{backend}'. Valid options: 'ollama', 'openai', 'vllm', 'azure', 'anthropic'.")


class RosaAgentNode(Node):
    """ROS 2 node wrapping ROSA agent with asynchronous LangChain event telemetry."""

    def __init__(self):
        super().__init__('rosa_agent')

        # Declare ROS parameters
        self.declare_parameter('llm_backend',             'vllm')
        self.declare_parameter('model',                   'Qwen/Qwen2.5-Coder-3B-Instruct-AWQ')
        self.declare_parameter('llm_base_url',            'http://localhost:8000/v1')
        self.declare_parameter('api_key',                 'sk-no-key-required')
        self.declare_parameter('temperature',             0.0)
        self.declare_parameter('ros_version',             2)
        self.declare_parameter('streaming',               True)
        self.declare_parameter('verbose',                 False)
        self.declare_parameter('max_iterations',          100)
        self.declare_parameter('accumulate_chat_history', True)

        backend     = self.get_parameter('llm_backend').value
        model       = self.get_parameter('model').value
        base_url    = self.get_parameter('llm_base_url').value
        api_key     = self.get_parameter('api_key').value
        temperature = self.get_parameter('temperature').value
        ros_version = self.get_parameter('ros_version').value
        streaming   = self.get_parameter('streaming').value
        verbose     = self.get_parameter('verbose').value
        max_iter    = self.get_parameter('max_iterations').value
        accumulate  = self.get_parameter('accumulate_chat_history').value

        self.get_logger().info(f'Initialising ROSA agent — backend={backend}, model={model}')

        try:
            llm = build_llm(backend, model, base_url, temperature, api_key)
        except (ImportError, ValueError) as exc:
            self.get_logger().fatal(str(exc))
            raise SystemExit(1)
        
        # Explicit instructions for native Qwen XML function syntax (double braces escaped for LangChain templates)
        #         qwen_prompts = RobotSystemPrompts(
        #             critical_instructions="""When invoking any tool or function, you MUST format your tool call strictly using native XML tags:
        # <function=tool_name>{{"arg_name": "arg_val"}}</function>
        # Do NOT use Markdown JSON code blocks for tool calls. Always execute functions with <function=tool_name>{{...}}</function>."""
        #         )

        # We don't need explicit instructions for XML function syntax
        # because LangChain's create_tool_calling_agent and vLLM handle the
        # OpenAI tool calling schema natively for Qwen.
        qwen_prompts = RobotSystemPrompts()

        try:
            self.agent = ROSA(
                ros_version=ros_version,
                llm=llm,
                prompts=qwen_prompts,
                streaming=streaming,
                verbose=verbose,
                max_iterations=max_iter,
                accumulate_chat_history=accumulate,
            )
        except Exception as exc:
            self.get_logger().fatal(f'Failed to initialise ROSA: {exc}')
            raise SystemExit(1)

        # Publishers and Subscriptions
        self._pub = self.create_publisher(String, 'rosa/response', 10)
        self._telemetry_pub = self.create_publisher(String, 'rosa/telemetry', 10)
        self._sub = self.create_subscription(String, 'rosa/query', self._on_query, 10)
        self._clear_srv = self.create_service(Empty, 'rosa/clear_history', self._on_clear_history)

        self._lock = threading.Lock()
        self._busy = False
        self.get_logger().info('ROSA Agent node running with non-blocking streaming telemetry.')

    def _publish_telemetry_event(self, event_data: dict) -> None:
        """Serialize event to JSON string and publish to /rosa/telemetry."""
        try:
            json_str = json.dumps(event_data, default=str)
            self._telemetry_pub.publish(String(data=json_str))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish telemetry event: {e}')

    def _on_query(self, msg: String) -> None:
        """Handle incoming query message immediately by spawning a non-blocking background worker thread."""
        if self._busy:
            self.get_logger().warn('ROSA is currently processing a query. Ignoring new request.')
            self._publish_telemetry_event({"type": "error", "content": "Agent is currently busy processing a command."})
            return

        # Spawn a standalone background worker thread so ROS 2 executor loop never blocks
        thread = threading.Thread(target=self._run_async_worker, args=(msg.data,), daemon=True)
        thread.start()

    def _run_async_worker(self, query: str) -> None:
        """Execute asynchronous LangChain streaming event pipeline inside worker thread."""
        with self._lock:
            self._busy = True
            self.get_logger().info(f'Query received: {query!r}')
            self._publish_telemetry_event({"type": "query_received", "query": query})
            try:
                # Create a dedicated asyncio event loop for LangChain astream_events v2
                asyncio.run(self._consume_event_stream(query))
            except Exception as exc:
                err_msg = f'[ROSA execution error] {exc}'
                self.get_logger().error(err_msg)
                self._publish_telemetry_event({"type": "error", "content": str(exc)})
                self._pub.publish(String(data=err_msg))
            finally:
                self._busy = False

    async def _consume_event_stream(self, query: str) -> None:
        """Iterate asynchronously through ROSA event stream (token generation, tool calls, results)."""
        final_text = ""
        async for event in self.agent.astream(query):
            event_type = event.get("type")
            
            if event_type == "tool_start":
                self.get_logger().info(f"[Tool Start] {event.get('name')} with inputs: {event.get('input')}")
                self._publish_telemetry_event(event)
            
            elif event_type == "tool_end":
                self.get_logger().info(f"[Tool End] {event.get('name')} completed")
                self._publish_telemetry_event(event)
            
            elif event_type == "token":
                # Send generated tokens to telemetry for interactive LLM effect
                self._publish_telemetry_event(event)
            
            elif event_type == "final":
                final_text = event.get("content", "")
                self._publish_telemetry_event(event)
                self._pub.publish(String(data=final_text))
                self.get_logger().info(f"Response completed and published ({len(final_text)} chars)")
            
            elif event_type == "error":
                self._publish_telemetry_event(event)
                self.get_logger().error(f"Stream error: {event.get('content')}")

    def _on_clear_history(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        """Service callback — clear agent memory and notify UI."""
        self.agent.clear_chat()
        self._publish_telemetry_event({"type": "clear", "content": "Chat history cleared."})
        self.get_logger().info('Chat history cleared.')
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RosaAgentNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
