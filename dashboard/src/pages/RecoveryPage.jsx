import React, { useState, useEffect } from 'react';
import { useRos } from '../context/RosContext';
import * as ROSLIB from 'roslib';
import { ShieldAlert, CheckCircle, Loader2, RotateCw } from 'lucide-react';
import './RecoveryPage.css';

export default function RecoveryPage() {
  const { ros, connectionStatus } = useRos();
  
  // States: 'IDLE', 'CANCELLING', 'SPINNING', 'DETECTING', 'DONE', 'ERROR'
  const [sequenceState, setSequenceState] = useState('IDLE');
  const [logs, setLogs] = useState([]);

  const addLog = (msg) => {
    setLogs(prev => [...prev, `[${new Date().toLocaleTimeString()}] ${msg}`]);
  };

  const startRecoverySequence = () => {
    if (!ros || connectionStatus !== 'CONNECTED') {
      addLog("ERROR: ROS not connected!");
      return;
    }
    setLogs([]);
    setSequenceState('CANCELLING');
    addLog("Step 1: Stopping current navigation tasks...");

    // Step 1: Cancel NavigateToPose (Standard way is calling cancel service)
    const cancelClient = new ROSLIB.Service({
      ros: ros,
      name: '/navigation/navigate_to_pose/_action/cancel_goal',
      serviceType: 'action_msgs/srv/CancelGoal'
    });

    // We attempt to cancel. Whether it succeeds, fails, or doesn't exist, we move on to Spin.
    // Use a plain object for the request payload and pass it as the first argument
    cancelClient.callService(
      {},
      (res) => {
        addLog("Navigation stopped successfully.");
        executeSpin();
      },
      (err) => {
        addLog("No active navigation to cancel (or cancel failed). Proceeding...");
        executeSpin();
      }
    );
  };

  const executeSpin = () => {
    setSequenceState('SPINNING');
    addLog("Step 2: Executing 360 Spin (Publishing to /cmd_vel manually)...");

    // roslibjs ActionClient doesn't support ROS 2 Action architecture natively
    // (it looks for ROS 1 topics instead of ROS 2 send_goal services).
    // Workaround: Publish directly to /cmd_vel for a set duration to achieve 360 deg.
    
    const cmdVelTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/navigation/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    });

    // Skoro Twój układ wspiera obrót w miejscu, używamy samego angular!
    const spinSpeed = 1; // rad/s
    
  // TRZEBA DOPASOWAĆ CZAS TRWANIA SPINU DO PRĘDKOŚCI OBROTU I WIELKOŚCI ROBOTA
    const durationMs = 40000; 

    const twistMsg = {
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: spinSpeed }
    };

    const stopMsg = {
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    };

    // Publish command at 10Hz
    const interval = setInterval(() => {
      cmdVelTopic.publish(twistMsg);
    }, 100);

    // Stop after duration
    setTimeout(() => {
      clearInterval(interval);
      cmdVelTopic.publish(stopMsg);
      addLog("Spin completed. Returned to original orientation.");
      executeLoopClosure();
    }, durationMs);
  };

  const executeLoopClosure = () => {
    setSequenceState('DETECTING');
    addLog("Step 3: Triggering RTAB-Map Deep Loop Closure Detection...");

    const detectService = new ROSLIB.Service({
      ros: ros,
      name: '/mapping/rtabmap_slam/detect_more_loop_closures',
      serviceType: 'std_srvs/srv/Empty'
    });

    detectService.callService(
      {},
      (res) => {
        addLog("Loop closures detected and graph optimized!");
        setSequenceState('DONE');
      },
      (err) => {
        addLog("Error triggering loop closure: " + err);
        setSequenceState('ERROR');
      }
    );
  };

  return (
    <div className="recovery-page" style={{ flex: 1, padding: '24px', overflowY: 'auto' }}>
      <div style={{ marginBottom: '24px' }}>
        <h1 style={{ margin: '0 0 8px 0', fontSize: '1.8rem', color: 'var(--status-amber)' }}>
          <ShieldAlert size={28} style={{ marginRight: '10px', verticalAlign: 'middle' }} />
          Emergency Recovery
        </h1>
        <p style={{ margin: 0, color: 'var(--text-muted)' }}>
          Execute automated sequences to recover from localization failures and map corruption.
        </p>
      </div>

      <div className="glass-panel recovery-card">
        <div className="recovery-header">
          <h2>Full Localization Fix</h2>
          <p>Sequence: Stop Nav &#8594; Spin 360° &#8594; Deep Loop Closure</p>
        </div>

        <div className="recovery-content">
          <button 
            className={`big-red-button ${sequenceState !== 'IDLE' && sequenceState !== 'DONE' && sequenceState !== 'ERROR' ? 'disabled' : ''}`}
            onClick={startRecoverySequence}
            disabled={sequenceState !== 'IDLE' && sequenceState !== 'DONE' && sequenceState !== 'ERROR'}
          >
            {sequenceState === 'IDLE' || sequenceState === 'DONE' || sequenceState === 'ERROR' ? (
              <>INITIATE RECOVERY SEQUENCE</>
            ) : (
              <><Loader2 className="spinner" size={20} /> SEQUENCE IN PROGRESS...</>
            )}
          </button>

          <div className="sequence-tracker">
            <div className={`track-step ${sequenceState === 'CANCELLING' ? 'active' : ''} ${['SPINNING', 'DETECTING', 'DONE'].includes(sequenceState) ? 'completed' : ''}`}>
              <div className="step-icon">1</div>
              <span>Stop Navigation</span>
            </div>
            <div className="track-line"></div>
            <div className={`track-step ${sequenceState === 'SPINNING' ? 'active' : ''} ${['DETECTING', 'DONE'].includes(sequenceState) ? 'completed' : ''}`}>
              <div className="step-icon">2</div>
              <span>360° LiDAR/Camera Spin</span>
            </div>
            <div className="track-line"></div>
            <div className={`track-step ${sequenceState === 'DETECTING' ? 'active' : ''} ${sequenceState === 'DONE' ? 'completed' : ''}`}>
              <div className="step-icon">3</div>
              <span>Deep Graph Optimization</span>
            </div>
          </div>
        </div>
      </div>

      <div className="glass-panel recovery-card" style={{ marginTop: '24px' }}>
        <div className="recovery-header">
          <h2>Path Recorder (Outdated Autonomy)</h2>
          <p>Requires Node: <code>ros2 run rover_autonomy_outdated path_recorder --ros-args -p odom_topic:=/kinematics/odom</code></p>
        </div>

        <div className="recovery-content" style={{ display: 'flex', gap: '16px', flexDirection: 'column' }}>
          <div style={{ display: 'flex', gap: '16px' }}>
            <button 
              className="orange-btn"
              style={{ flex: 1, padding: '16px', background: 'linear-gradient(135deg, #f59e0b, #d97706)' }}
              onClick={() => {
                if (!ros) { addLog("ERROR: ROS not connected!"); return; }
                addLog("Triggering Path Recorder Escape...");
                const svc = new ROSLIB.Service({ ros: ros, name: '/rover_recovery/escape', serviceType: 'std_srvs/srv/Trigger' });
                svc.callService({}, 
                  (res) => addLog("Escape triggered successfully! " + (res.message || '')), 
                  (err) => addLog("Escape failed: " + err)
                );
              }}
            >
              TRIGGER ESCAPE (REVERSE PATH)
            </button>
            <button 
              className="orange-btn"
              style={{ flex: 1, padding: '16px', background: 'linear-gradient(135deg, #ef4444, #b91c1c)' }}
              onClick={() => {
                if (!ros) { addLog("ERROR: ROS not connected!"); return; }
                addLog("Clearing Recorded Path...");
                const svc = new ROSLIB.Service({ ros: ros, name: '/rover_recovery/clear', serviceType: 'std_srvs/srv/Trigger' });
                svc.callService({}, 
                  (res) => addLog("Path cleared successfully! " + (res.message || '')), 
                  (err) => {
                    // Fallback to std_srvs/srv/Empty if Trigger fails
                    const emptySvc = new ROSLIB.Service({ ros: ros, name: '/rover_recovery/clear', serviceType: 'std_srvs/srv/Empty' });
                    emptySvc.callService({}, 
                      () => addLog("Path cleared successfully (via Empty)!"),
                      (e) => addLog("Clear path failed: " + e)
                    );
                  }
                );
              }}
            >
              CLEAR RECORDED PATH
            </button>
          </div>
        </div>
      </div>

      {logs.length > 0 && (
        <div className="recovery-console">
          <div className="console-header">Recovery Logs</div>
          <div className="console-output">
            {logs.map((log, i) => (
              <div key={i} className="log-line">{log}</div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
