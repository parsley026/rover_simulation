import React, { useState, useEffect } from 'react';
import { useRos } from '../context/RosContext';
import { Terminal, Code, ChevronDown, ChevronUp, CheckCircle2, PlayCircle, Trash2, X } from 'lucide-react';

export default function TelemetryInspector() {
  const { connectionStatus, getTopic } = useRos();
  const [toolEvents, setToolEvents] = useState([
    {
      id: 'init_sample',
      type: 'tool_end',
      name: 'ros2_topic_list',
      input: '{}',
      output: '["/rosa/query", "/rosa/response", "/rosa/telemetry", "/rosa/system_status", "/parameter_events"]',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }),
      expanded: true
    }
  ]);

  useEffect(() => {
    if (connectionStatus !== 'CONNECTED') return;

    const telemetryTopic = getTopic('/rosa/telemetry', 'std_msgs/String');
    if (!telemetryTopic) return;

    const handleTelemetry = (msg) => {
      try {
        const event = JSON.parse(msg.data);
        const timestamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });

        if (event.type === 'tool_start') {
          setToolEvents(prev => [
            {
              id: 'tool_' + Date.now(),
              type: 'tool_start',
              name: event.name || 'unknown_tool',
              input: typeof event.input === 'object' ? JSON.stringify(event.input, null, 2) : String(event.input || ''),
              output: null,
              timestamp,
              expanded: true
            },
            ...prev
          ]);
        } else if (event.type === 'tool_end') {
          // Find matching open tool card and populate output
          setToolEvents(prev => {
            const index = prev.findIndex(t => t.name === event.name && t.type === 'tool_start' && t.output === null);
            if (index !== -1) {
              const updated = [...prev];
              updated[index] = {
                ...updated[index],
                type: 'tool_end',
                output: typeof event.output === 'object' ? JSON.stringify(event.output, null, 2) : String(event.output || '')
              };
              return updated;
            } else {
              // Create standalone end card if no open start found
              return [
                {
                  id: 'tool_end_' + Date.now(),
                  type: 'tool_end',
                  name: event.name || 'unknown_tool',
                  input: 'Executed',
                  output: typeof event.output === 'object' ? JSON.stringify(event.output, null, 2) : String(event.output || ''),
                  timestamp,
                  expanded: true
                },
                ...prev
              ];
            }
          });
        }
      } catch (e) {
        console.debug('Inspector parse note:', e);
      }
    };

    telemetryTopic.subscribe(handleTelemetry);
    return () => {
      telemetryTopic.unsubscribe(handleTelemetry);
    };
  }, [connectionStatus, getTopic]);

  const toggleExpand = (id) => {
    setToolEvents(prev => prev.map(t => t.id === id ? { ...t, expanded: !t.expanded } : t));
  };

  const deleteLog = (id, e) => {
    e.stopPropagation();
    setToolEvents(prev => prev.filter(t => t.id !== id));
  };

  const clearLogs = () => {
    setToolEvents([]);
  };

  const formatJsonOrText = (str) => {
    if (!str) return 'Waiting for return payload...';
    try {
      const parsed = JSON.parse(str);
      return JSON.stringify(parsed, null, 2);
    } catch (e) {
      return str;
    }
  };

  return (
    <aside className="telemetry-section glass-panel">
      <div className="section-header">
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <Terminal size={18} style={{ color: 'var(--accent-cyan)' }} />
          <span>Agent Telemetry</span>
          <span style={{ fontSize: '0.75rem', background: 'rgba(0, 242, 254, 0.1)', color: 'var(--accent-cyan)', padding: '2px 8px', borderRadius: '12px' }}>
            {toolEvents.length}
          </span>
        </div>
        <button 
          onClick={clearLogs} 
          style={{ background: 'transparent', border: 'none', color: 'var(--text-muted)', cursor: 'pointer', display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.82rem' }}
          title="Clear all telemetry tool logs"
        >
          <Trash2 size={15} /> Clear Logs
        </button>
      </div>

      <div className="messages-container" style={{ gap: '12px' }}>
        {toolEvents.length === 0 ? (
          <div style={{ textAlign: 'center', color: 'var(--text-muted)', fontSize: '0.88rem', marginTop: '40px' }}>
            <Code size={32} style={{ opacity: 0.3, margin: '0 auto 12px auto' }} />
            No tool executions recorded yet.<br />When ROSA issues robotics tools, live parameters and return payloads appear here.
          </div>
        ) : (
          toolEvents.map((t) => (
            <div key={t.id} className="telemetry-item">
              <div className="telemetry-header" onClick={() => toggleExpand(t.id)}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px', flex: 1, overflow: 'hidden' }}>
                  {t.output === null ? (
                    <PlayCircle size={15} style={{ color: 'var(--status-amber)' }} />
                  ) : (
                    <CheckCircle2 size={15} style={{ color: 'var(--status-emerald)' }} />
                  )}
                  <span style={{ textOverflow: 'ellipsis', overflow: 'hidden', whiteSpace: 'nowrap' }}>{t.name}</span>
                  <small style={{ color: 'var(--text-muted)', fontSize: '0.75rem', fontWeight: 400 }}>[{t.timestamp}]</small>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  {t.expanded ? <ChevronUp size={16} /> : <ChevronDown size={16} />}
                  <button 
                    onClick={(e) => deleteLog(t.id, e)} 
                    style={{ background: 'transparent', border: 'none', color: 'var(--text-muted)', cursor: 'pointer', display: 'flex', alignItems: 'center', padding: '2px', borderRadius: '4px' }}
                    title="Delete log"
                    onMouseEnter={(e) => e.currentTarget.style.color = 'var(--status-rose)'}
                    onMouseLeave={(e) => e.currentTarget.style.color = 'var(--text-muted)'}
                  >
                    <X size={15} />
                  </button>
                </div>
              </div>

              {t.expanded && (
                <div className="telemetry-content">
                  <div style={{ color: 'var(--text-muted)', marginBottom: '4px', fontSize: '0.75rem' }}>// arguments & inputs:</div>
                  <pre style={{ margin: '0 0 10px 0', color: 'var(--accent-blue)' }}>{formatJsonOrText(t.input)}</pre>
                  
                  <div style={{ color: 'var(--text-muted)', marginBottom: '4px', fontSize: '0.75rem' }}>// raw ros2 return data:</div>
                  <pre style={{ margin: 0, color: t.output === null ? 'var(--status-amber)' : 'var(--status-emerald)' }}>
                    {formatJsonOrText(t.output)}
                  </pre>
                </div>
              )}
            </div>
          ))
        )}
      </div>
    </aside>
  );
}
