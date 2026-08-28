import React, { useEffect, useState } from 'react';
import { NavLink } from 'react-router-dom';
import { useRos } from '../context/RosContext';
import { Cpu, HardDrive, Zap, Radio, AlertCircle, Settings, MapPin } from 'lucide-react';

export default function Navbar() {
  const { connectionStatus, getTopic, reconnect } = useRos();
  const [metrics, setMetrics] = useState({
    gpu: { gpu_name: "RTX 3060", vram_used_mb: 0, vram_total_mb: 6144, vram_percent: 0.0, gpu_temp_c: 45.0 },
    system: { cpu_percent: 0.0, ram_percent: 0.0 },
    rover: { battery_voltage: 24.6, battery_percent: 98.0, battery_status: "ONLINE" }
  });

  useEffect(() => {
    if (connectionStatus !== 'CONNECTED') return;

    const sysTopic = getTopic('/rosa/system_status', 'std_msgs/String');
    if (!sysTopic) return;

    const handleMessage = (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setMetrics(data);
      } catch (e) {
        console.debug('Failed to parse system metric JSON:', e);
      }
    };

    sysTopic.subscribe(handleMessage);

    return () => {
      sysTopic.unsubscribe(handleMessage);
    };
  }, [connectionStatus, getTopic]);

  const getStatusDotClass = () => {
    if (connectionStatus === 'CONNECTED') return 'online';
    if (connectionStatus === 'CONNECTING') return 'connecting';
    return 'offline';
  };

  const vramColor = metrics.gpu.vram_percent > 85 ? 'var(--status-rose)' : metrics.gpu.vram_percent > 65 ? 'var(--status-amber)' : 'var(--accent-cyan)';
  const batteryColor = metrics.rover.battery_percent < 25 ? 'var(--status-rose)' : 'var(--status-emerald)';

  return (
    <header className="status-navbar">
      <div className="nav-brand">
        <Radio size={24} style={{ color: 'var(--accent-cyan)' }} />
        <span>ROSA Command Center <small style={{ fontWeight: 400, fontSize: '0.75rem', color: 'var(--text-muted)' }}>AI-First Stage 1</small></span>
      </div>

      {/* Page Navigation */}
      <nav style={{ display: 'flex', gap: '4px' }}>
        <NavLink
          to="/"
          end
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
            background: isActive ? 'rgba(0,242,254,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(0,242,254,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          Dashboard
        </NavLink>
        <NavLink
          to="/config"
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
            background: isActive ? 'rgba(0,242,254,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(0,242,254,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          <Settings size={15} />
          Config
        </NavLink>
        <NavLink
          to="/map"
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
            background: isActive ? 'rgba(0,242,254,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(0,242,254,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          Map
        </NavLink>
        <NavLink
          to="/navigation"
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
            background: isActive ? 'rgba(0,242,254,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(0,242,254,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          <MapPin size={15} />
          Navigation
        </NavLink>
        <NavLink
          to="/localization"
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
            background: isActive ? 'rgba(0,242,254,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(0,242,254,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          <MapPin size={15} />
          Localization
        </NavLink>
        <NavLink
          to="/recovery"
          style={({ isActive }) => ({
            display: 'flex', alignItems: 'center', gap: '6px',
            padding: '6px 14px', borderRadius: '8px',
            textDecoration: 'none', fontWeight: 500, fontSize: '0.88rem',
            color: isActive ? '#f87171' : 'var(--text-muted)',
            background: isActive ? 'rgba(248,113,113,0.1)' : 'transparent',
            border: `1px solid ${isActive ? 'rgba(248,113,113,0.25)' : 'transparent'}`,
            transition: 'all 0.2s ease',
          })}
        >
          <AlertCircle size={15} />
          Recovery
        </NavLink>
      </nav>

      <div className="nav-metrics">
        {/* GPU VRAM Metric Pill (RTX 3060) */}
        <div className="metric-pill" title={`NVIDIA RTX 3060 Temp: ${metrics.gpu.gpu_temp_c}°C`}>
          <HardDrive size={16} color={vramColor} />
          <span>VRAM <strong>{metrics.gpu.vram_used_mb}</strong> / {metrics.gpu.vram_total_mb} MB</span>
          <div className="progress-container">
            <div className="progress-fill" style={{ width: `${Math.min(100, metrics.gpu.vram_percent)}%`, background: vramColor }} />
          </div>
        </div>

        {/* CPU Load Metric Pill */}
        <div className="metric-pill">
          <Cpu size={16} style={{ color: 'var(--accent-blue)' }} />
          <span>CPU <strong>{metrics.system.cpu_percent}%</strong></span>
          <div className="progress-container">
            <div className="progress-fill" style={{ width: `${Math.min(100, metrics.system.cpu_percent)}%`, background: 'var(--accent-blue)' }} />
          </div>
        </div>

        {/* Simulated Rover Battery Gauge */}
        <div className="metric-pill">
          <Zap size={16} color={batteryColor} />
          <span>Rover <strong>{metrics.rover.battery_voltage}V</strong> ({metrics.rover.battery_percent}%)</span>
        </div>

        {/* WebSocket Bridge Heartbeat Indicator */}
        <div className="metric-pill" style={{ cursor: connectionStatus === 'ERROR' || connectionStatus === 'CLOSED' ? 'pointer' : 'default' }} onClick={reconnect}>
          <span className={`status-dot ${getStatusDotClass()}`} />
          <span style={{ fontSize: '0.82rem', fontWeight: 600 }}>
            {connectionStatus === 'CONNECTED' ? 'rosbridge : 9090' : connectionStatus}
          </span>
          {connectionStatus !== 'CONNECTED' && <AlertCircle size={14} style={{ color: 'var(--status-amber)' }} />}
        </div>
      </div>
    </header>
  );
}
