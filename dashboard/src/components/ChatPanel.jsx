import React, { useState, useEffect, useRef } from 'react';
import { useRos } from '../context/RosContext';
import { Send, Bot, User, Trash2, Sparkles, RefreshCw } from 'lucide-react';
import * as ROSLIB from 'roslib';

export default function ChatPanel() {
  const { ros, connectionStatus, getTopic, publish } = useRos();
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState([
    {
      id: 'welcome_1',
      sender: 'agent',
      text: "Hello! I am ROSA, your intelligent Qwen-powered robotic copilot. Ask me anything about our ROS 2 topics, simulation nodes, kinematics, or diagnostic metrics.",
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      status: 'idle'
    }
  ]);
  const [isBusy, setIsBusy] = useState(false);
  const [activeTool, setActiveTool] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isBusy, activeTool]);

  useEffect(() => {
    if (connectionStatus !== 'CONNECTED') return;

    // Listen to streaming tokens and tool start notifications on telemetry topic
    const telemetryTopic = getTopic('/rosa/telemetry', 'std_msgs/String');
    const respTopic = getTopic('/rosa/response', 'std_msgs/String');
    if (!telemetryTopic || !respTopic) return;

    const handleTelemetry = (msg) => {
      try {
        const event = JSON.parse(msg.data);
        if (event.type === 'tool_start') {
          setActiveTool(event.name);
        } else if (event.type === 'tool_end') {
          setActiveTool(null);
        } else if (event.type === 'token') {
          // Append streaming tokens directly to the active assistant message if present
          setMessages(prev => {
            const last = prev[prev.length - 1];
            if (last && last.sender === 'agent' && last.status === 'streaming') {
              return [
                ...prev.slice(0, prev.length - 1),
                { ...last, text: last.text + (event.content || '') }
              ];
            }
            return prev;
          });
        } else if (event.type === 'clear') {
          setMessages([{
            id: 'cleared_' + Date.now(),
            sender: 'agent',
            text: 'Chat history cleared. How can I help you next?',
            timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
            status: 'idle'
          }]);
        }
      } catch (e) {
        console.debug('Telemetry parse note:', e);
      }
    };

    const handleResponse = (msg) => {
      setIsBusy(false);
      setActiveTool(null);
      setMessages(prev => {
        const last = prev[prev.length - 1];
        if (last && last.sender === 'agent' && last.status === 'streaming') {
          // Finalize streaming bubble with complete text
          return [
            ...prev.slice(0, prev.length - 1),
            { ...last, text: msg.data, status: 'idle' }
          ];
        } else {
          return [
            ...prev,
            {
              id: 'resp_' + Date.now(),
              sender: 'agent',
              text: msg.data,
              timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
              status: 'idle'
            }
          ];
        }
      });
    };

    telemetryTopic.subscribe(handleTelemetry);
    respTopic.subscribe(handleResponse);

    return () => {
      telemetryTopic.unsubscribe(handleTelemetry);
      respTopic.unsubscribe(handleResponse);
    };
  }, [connectionStatus, getTopic]);

  const handleSend = (textToSend = input) => {
    if (!textToSend.trim() || connectionStatus !== 'CONNECTED' || isBusy) return;

    const userMsg = {
      id: 'usr_' + Date.now(),
      sender: 'user',
      text: textToSend,
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    // Prepare placeholder streaming assistant bubble
    const agentMsg = {
      id: 'agent_stream_' + Date.now(),
      sender: 'agent',
      text: '',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      status: 'streaming'
    };

    setMessages(prev => [...prev, userMsg, agentMsg]);
    setInput('');
    setIsBusy(true);

    publish('/rosa/query', 'std_msgs/String', { data: textToSend });
  };

  const handleClearHistory = () => {
    if (ros && ros.isConnected) {
      const srv = new ROSLIB.Service({ ros, name: '/rosa/clear_history', serviceType: 'std_srvs/Empty' });
      srv.callService({}, () => {
        setIsBusy(false);
      });
    }
  };

  const macroQueries = [
    "List all active ROS 2 topics",
    "Check running simulation nodes",
    "Run diagnostics on system tools",
    "Explain our rover kinematics setup"
  ];

  return (
    <section className="chat-section glass-panel">
      <div className="section-header">
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <Bot size={20} style={{ color: 'var(--accent-cyan)' }} />
          <span>ROSA Conversational Copilot</span>
        </div>
        <button 
          onClick={handleClearHistory} 
          style={{ background: 'transparent', border: 'none', color: 'var(--text-muted)', cursor: 'pointer', display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.82rem' }}
          title="Clear agent memory history"
        >
          <Trash2 size={15} /> Clear Chat
        </button>
      </div>

      <div className="messages-container">
        {messages.map((m) => (
          <div key={m.id} className={`message-card ${m.sender}`}>
            <div className="message-meta">
              {m.sender === 'user' ? <User size={12} /> : <Bot size={12} style={{ color: 'var(--accent-cyan)' }} />}
              <span>{m.sender === 'user' ? 'Operator' : 'ROSA'}</span>
              <span>•</span>
              <span>{m.timestamp}</span>
            </div>
            <div className="message-bubble">
              {m.text || (m.status === 'streaming' && (
                <span style={{ color: 'var(--text-muted)', fontStyle: 'italic', display: 'flex', alignItems: 'center', gap: '8px' }}>
                  <RefreshCw size={14} className="spin-icon" style={{ animation: 'spin 1.5s linear infinite' }} />
                  {activeTool ? `Executing robot tool: [${activeTool}]...` : "Qwen is synthesizing a response..."}
                </span>
              ))}
            </div>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Quick Action Macro Chips */}
      <div className="macro-chips">
        <Sparkles size={14} style={{ color: 'var(--accent-cyan)', alignSelf: 'center', marginRight: '4px' }} />
        {macroQueries.map((mq, i) => (
          <button key={i} className="macro-chip" onClick={() => handleSend(mq)} disabled={isBusy || connectionStatus !== 'CONNECTED'}>
            {mq}
          </button>
        ))}
      </div>

      <div className="input-area">
        <input 
          type="text"
          className="chat-input"
          placeholder={connectionStatus === 'CONNECTED' ? (isBusy ? "ROSA is thinking or executing commands..." : "Ask ROSA to control or inspect the rover...") : "Waiting for WebSocket gateway connection..."}
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={(e) => e.key === 'Enter' && handleSend()}
          disabled={isBusy || connectionStatus !== 'CONNECTED'}
        />
        <button className="send-button glow-border" onClick={() => handleSend()} disabled={isBusy || connectionStatus !== 'CONNECTED' || !input.trim()}>
          <Send size={16} /> Send
        </button>
      </div>
      <style>{`
        @keyframes spin { 100% { transform: rotate(360deg); } }
      `}</style>
    </section>
  );
}
