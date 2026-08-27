import React, { useState } from 'react';
import { useRos } from '../context/RosContext';
import * as ROSLIB from 'roslib';
import { Navigation, Target, CheckCircle2, AlertCircle, RotateCcw, XCircle } from 'lucide-react';

// Convert yaw (degrees) to quaternion z/w components (rotation around Z axis)
function yawToQuaternion(yawDeg) {
  const yaw = (yawDeg * Math.PI) / 180;
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2),
  };
}

function PoseForm({ title, icon: Icon, accentColor, onSubmit, submitLabel, disabled }) {
  const [x, setX] = useState('0.0');
  const [y, setY] = useState('0.0');
  const [yaw, setYaw] = useState('0');
  const [feedback, setFeedback] = useState(null); // { type: 'ok'|'err', msg }

  const handleSubmit = (e) => {
    e.preventDefault();
    const px = parseFloat(x);
    const py = parseFloat(y);
    const pyaw = parseFloat(yaw);
    if (isNaN(px) || isNaN(py) || isNaN(pyaw)) {
      setFeedback({ type: 'err', msg: 'Wszystkie pola muszą być liczbami.' });
      return;
    }
    const quat = yawToQuaternion(pyaw);
    onSubmit({ x: px, y: py, quat });
    setFeedback({ type: 'ok', msg: `Wysłano: x=${px}, y=${py}, yaw=${pyaw}°` });
    setTimeout(() => setFeedback(null), 3500);
  };

  const handleReset = () => {
    setX('0.0');
    setY('0.0');
    setYaw('0');
    setFeedback(null);
  };

  return (
    <div className="nav2-form-card glass-panel" style={{ border: `1px solid ${accentColor}33`, marginBottom: '20px' }}>
      {/* Card Header */}
      <div style={{
        padding: '14px 20px',
        borderBottom: `1px solid ${accentColor}22`,
        display: 'flex',
        alignItems: 'center',
        gap: '10px',
        background: `${accentColor}0a`,
        borderRadius: '12px 12px 0 0',
      }}>
        <Icon size={20} color={accentColor} />
        <span style={{ fontWeight: 600, fontSize: '1rem', color: accentColor }}>{title}</span>
      </div>

      {/* Form Body */}
      <form onSubmit={handleSubmit} style={{ padding: '20px', display: 'flex', flexDirection: 'column', gap: '16px' }}>
        {/* X / Y Row */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
          <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
            <span style={{ fontSize: '0.78rem', fontWeight: 600, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.06em' }}>
              X (m)
            </span>
            <input
              type="number"
              step="0.01"
              value={x}
              onChange={(e) => setX(e.target.value)}
              className="nav2-input"
              placeholder="0.0"
              disabled={disabled}
            />
          </label>
          <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
            <span style={{ fontSize: '0.78rem', fontWeight: 600, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.06em' }}>
              Y (m)
            </span>
            <input
              type="number"
              step="0.01"
              value={y}
              onChange={(e) => setY(e.target.value)}
              className="nav2-input"
              placeholder="0.0"
              disabled={disabled}
            />
          </label>
        </div>

        {/* Yaw Row */}
        <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <span style={{ fontSize: '0.78rem', fontWeight: 600, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.06em' }}>
            Orientacja — Yaw (°)
          </span>
          <div style={{ position: 'relative' }}>
            <input
              type="number"
              step="1"
              min="-180"
              max="180"
              value={yaw}
              onChange={(e) => setYaw(e.target.value)}
              className="nav2-input"
              placeholder="0"
              disabled={disabled}
              style={{ width: '100%' }}
            />
            <span style={{
              position: 'absolute', right: '14px', top: '50%', transform: 'translateY(-50%)',
              fontSize: '0.8rem', color: 'var(--text-muted)', pointerEvents: 'none'
            }}>
              {parseFloat(yaw) || 0}° → q_z={yawToQuaternion(parseFloat(yaw) || 0).z.toFixed(3)}
            </span>
          </div>

          {/* Yaw slider */}
          <input
            type="range"
            min="-180"
            max="180"
            step="1"
            value={yaw}
            onChange={(e) => setYaw(e.target.value)}
            disabled={disabled}
            style={{ accentColor, marginTop: '4px' }}
          />
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.72rem', color: 'var(--text-muted)' }}>
            <span>-180°</span><span>0°</span><span>+180°</span>
          </div>
        </label>

        {/* Quaternion preview */}
        <div style={{
          background: 'rgba(0,0,0,0.3)',
          border: '1px solid rgba(255,255,255,0.06)',
          borderRadius: '8px',
          padding: '10px 14px',
          fontFamily: 'var(--font-mono)',
          fontSize: '0.78rem',
          color: accentColor,
        }}>
          {(() => {
            const q = yawToQuaternion(parseFloat(yaw) || 0);
            return `quaternion → x:${q.x} y:${q.y} z:${q.z.toFixed(4)} w:${q.w.toFixed(4)}`;
          })()}
        </div>

        {/* Feedback Banner */}
        {feedback && (
          <div style={{
            display: 'flex', alignItems: 'center', gap: '8px',
            padding: '10px 14px', borderRadius: '8px',
            background: feedback.type === 'ok' ? 'rgba(16,185,129,0.12)' : 'rgba(244,63,94,0.12)',
            border: `1px solid ${feedback.type === 'ok' ? 'var(--status-emerald)' : 'var(--status-rose)'}44`,
            fontSize: '0.85rem',
            color: feedback.type === 'ok' ? 'var(--status-emerald)' : 'var(--status-rose)',
          }}>
            {feedback.type === 'ok'
              ? <CheckCircle2 size={16} />
              : <AlertCircle size={16} />
            }
            {feedback.msg}
          </div>
        )}

        {/* Action Buttons */}
        <div style={{ display: 'flex', gap: '10px' }}>
          <button
            type="submit"
            disabled={disabled}
            style={{
              flex: 1,
              background: disabled ? 'rgba(255,255,255,0.05)' : `linear-gradient(135deg, ${accentColor}, ${accentColor}99)`,
              border: 'none',
              borderRadius: '8px',
              padding: '11px 20px',
              color: disabled ? 'var(--text-muted)' : '#000',
              fontWeight: 700,
              fontSize: '0.9rem',
              cursor: disabled ? 'not-allowed' : 'pointer',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              gap: '8px',
              transition: 'all 0.2s ease',
            }}
          >
            <Icon size={16} />
            {submitLabel}
          </button>
          <button
            type="button"
            onClick={handleReset}
            title="Reset do zera"
            style={{
              background: 'rgba(255,255,255,0.04)',
              border: '1px solid rgba(255,255,255,0.1)',
              borderRadius: '8px',
              padding: '11px 14px',
              color: 'var(--text-muted)',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              transition: 'all 0.2s ease',
            }}
            onMouseEnter={(e) => { e.currentTarget.style.borderColor = accentColor; e.currentTarget.style.color = accentColor; }}
            onMouseLeave={(e) => { e.currentTarget.style.borderColor = 'rgba(255,255,255,0.1)'; e.currentTarget.style.color = 'var(--text-muted)'; }}
          >
            <RotateCcw size={16} />
          </button>
        </div>

        {disabled && (
          <p style={{ textAlign: 'center', fontSize: '0.82rem', color: 'var(--status-amber)', margin: 0 }}>
            ⚠ Brak połączenia z rosbridge — podłącz się, aby wysłać komendę.
          </p>
        )}
      </form>
    </div>
  );
}

function CancelButton({ disabled, onCancel, cancelFeedback }) {
  const accentColor = 'var(--status-rose)';

  return (
    <div className="nav2-form-card glass-panel" style={{ border: `1px solid ${accentColor}33`, marginBottom: '20px' }}>
      <div style={{
        padding: '14px 20px',
        borderBottom: `1px solid ${accentColor}22`,
        display: 'flex',
        alignItems: 'center',
        gap: '10px',
        background: `${accentColor}0a`,
        borderRadius: '12px 12px 0 0',
      }}>
        <XCircle size={20} color={accentColor} />
        <span style={{ fontWeight: 600, fontSize: '1rem', color: accentColor }}>Cancel Navigation</span>
      </div>

      <div style={{ padding: '20px', display: 'flex', flexDirection: 'column', gap: '14px' }}>
        <p style={{ color: 'var(--text-muted)', fontSize: '0.85rem', margin: 0 }}>
          Anuluj aktualnie wykonywaną akcję nawigacji (navigate_to_pose / waypoint follower).
        </p>

        {cancelFeedback && (
          <div style={{
            display: 'flex', alignItems: 'center', gap: '8px',
            padding: '10px 14px', borderRadius: '8px',
            background: cancelFeedback.type === 'ok' ? 'rgba(16,185,129,0.12)' : 'rgba(244,63,94,0.12)',
            border: `1px solid ${cancelFeedback.type === 'ok' ? 'var(--status-emerald)' : 'var(--status-rose)'}44`,
            fontSize: '0.85rem',
            color: cancelFeedback.type === 'ok' ? 'var(--status-emerald)' : 'var(--status-rose)',
          }}>
            {cancelFeedback.type === 'ok' ? <CheckCircle2 size={16} /> : <AlertCircle size={16} />}
            {cancelFeedback.msg}
          </div>
        )}

        <button
          type="button"
          onClick={onCancel}
          disabled={disabled}
          style={{
            width: '100%',
            background: disabled
              ? 'rgba(255,255,255,0.05)'
              : 'linear-gradient(135deg, #f43f5e, #e11d48)',
            border: 'none',
            borderRadius: '8px',
            padding: '13px 20px',
            color: disabled ? 'var(--text-muted)' : '#fff',
            fontWeight: 700,
            fontSize: '0.95rem',
            cursor: disabled ? 'not-allowed' : 'pointer',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: '10px',
            transition: 'all 0.2s ease',
          }}
        >
          <XCircle size={18} />
          Cancel Action
        </button>

        {disabled && (
          <p style={{ textAlign: 'center', fontSize: '0.82rem', color: 'var(--status-amber)', margin: 0 }}>
            ⚠ Brak połączenia z rosbridge.
          </p>
        )}
      </div>
    </div>
  );
}

export default function Nav2Control() {
  const { connectionStatus, publish, getRosTime } = useRos();
  const isConnected = connectionStatus === 'CONNECTED';
  const [cancelFeedback, setCancelFeedback] = useState(null);

  // Publish /initialpose  (geometry_msgs/PoseWithCovarianceStamped)
  const sendInitialPose = ({ x, y, quat }) => {
    const stamp = getRosTime();
    publish('/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', {
      header: { stamp, frame_id: 'map' },
      pose: {
        pose: {
          position: { x, y, z: 0.0 },
          orientation: { ...quat },
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0.06853891945200942,
        ],
      },
    });
  };

  // Publish /goal_pose  (geometry_msgs/PoseStamped)
  const sendGoalPose = ({ x, y, quat }) => {
    const stamp = getRosTime();
    publish('/goal_pose', 'geometry_msgs/PoseStamped', {
      header: { stamp, frame_id: 'map' },
      pose: {
        position: { x, y, z: 0.0 },
        orientation: { ...quat },
      },
    });
  };

  // Cancel all running navigation goals
  const cancelNavigation = () => {
    const cancelServices = [
      '/navigate_to_pose/_action/cancel_goal',
      '/follow_waypoints/_action/cancel_goal',
    ];
    const cancelReq = {
      goal_info: {
        goal_id: { uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] },
        stamp: { sec: 0, nanosec: 0 }
      }
    };

    let called = false;
    cancelServices.forEach(svcName => {
      if (ros) {
        const svc = new ROSLIB.Service({
          ros: ros,
          name: svcName,
          serviceType: 'action_msgs/srv/CancelGoal'
        });
        svc.callService(cancelReq, () => {}, () => {});
        called = true;
      }
    });

    if (called) {
      setCancelFeedback({ type: 'ok', msg: 'Wysłano cancel — nawigacja powinna się zatrzymać.' });
    } else {
      setCancelFeedback({ type: 'err', msg: 'Brak połączenia z ROS.' });
    }
    setTimeout(() => setCancelFeedback(null), 4000);
  };

  return (
    <div style={{ padding: '24px', maxWidth: '700px', margin: '0 auto', width: '100%' }}>
      {/* Section Title */}
      <div style={{ marginBottom: '28px' }}>
        <h2 style={{
          fontSize: '1.3rem', fontWeight: 700, margin: '0 0 6px 0',
          background: 'linear-gradient(135deg, var(--accent-cyan), var(--accent-blue))',
          WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent',
        }}>
          Nav2 Control
        </h2>
        <p style={{ color: 'var(--text-muted)', fontSize: '0.88rem', margin: 0 }}>
          Ustaw pozycję startową łazika lub wyślij cel nawigacji przez ROS2 Nav2.
          Upewnij się, że Nav2 i AMCL są uruchomione.
        </p>
      </div>

      {/* Initial Pose */}
      <PoseForm
        title="Set Initial Pose"
        icon={Navigation}
        accentColor="var(--accent-cyan)"
        onSubmit={sendInitialPose}
        submitLabel="Wyślij Initial Pose"
        disabled={!isConnected}
      />

      {/* Goal Pose */}
      <PoseForm
        title="Set Navigation Goal"
        icon={Target}
        accentColor="var(--accent-purple)"
        onSubmit={sendGoalPose}
        submitLabel="Wyślij Goal Pose"
        disabled={!isConnected}
      />

      {/* Cancel Navigation */}
      <CancelButton
        disabled={!isConnected}
        onCancel={cancelNavigation}
        cancelFeedback={cancelFeedback}
      />

      {/* ROS Topic Info */}
      <div style={{
        background: 'rgba(0,0,0,0.25)',
        border: '1px solid rgba(255,255,255,0.07)',
        borderRadius: '10px',
        padding: '14px 18px',
        fontFamily: 'var(--font-mono)',
        fontSize: '0.78rem',
        color: 'var(--text-muted)',
        lineHeight: '1.8',
      }}>
        <div style={{ color: 'var(--text-muted)', marginBottom: '6px', fontSize: '0.72rem', textTransform: 'uppercase', letterSpacing: '0.08em' }}>ROS2 Topics</div>
        <div>
          <span style={{ color: 'var(--accent-cyan)' }}>/initialpose</span>
          {'  '}geometry_msgs/PoseWithCovarianceStamped
        </div>
        <div>
          <span style={{ color: 'var(--accent-purple)' }}>/goal_pose</span>
          {'     '}geometry_msgs/PoseStamped
        </div>
        <div>
          <span style={{ color: 'var(--status-rose)' }}>/navigate_to_pose/_action/cancel_goal</span>
          {'  '}action_msgs/CancelGoal
        </div>
        <div style={{ marginTop: '6px' }}>frame_id: <span style={{ color: 'var(--status-emerald)' }}>map</span></div>
      </div>
    </div>
  );
}
