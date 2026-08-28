import React, { useState } from 'react';
import { useRos } from '../context/RosContext';
import * as ROSLIB from 'roslib';
import { Info, ChevronDown, XCircle } from 'lucide-react';
import './MapPage.css';

const CollapsibleCategory = ({ title, subtitle, children }) => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div>
      <div className="accordion-header" onClick={() => setIsOpen(!isOpen)}>
        <div className="accordion-title-group">
          <h2 className="accordion-title">{title}</h2>
          <p className="accordion-subtitle">{subtitle}</p>
        </div>
        <ChevronDown size={24} className={`accordion-icon ${isOpen ? 'open' : ''}`} />
      </div>
      {isOpen && (
        <div className="accordion-content">
          {children}
        </div>
      )}
    </div>
  );
};

function yawToQuaternion(yawDegrees) {
    const yaw = yawDegrees * Math.PI / 180.0;
    return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
}

const ActionRow = ({ title, actionName, proxyService, needsCoordinates, needsMultipleCoordinates, longDesc, behaviorUse, onHover }) => {
  const { ros, connectionStatus } = useRos();
  const [status, setStatus] = useState('');

  // Single coordinate state
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [yaw, setYaw] = useState(0);

  // Multiple coordinates state
  const [waypoints, setWaypoints] = useState([{x: 0, y: 0, yaw: 0}]);

  const handleFireAction = () => {
    if (!ros || connectionStatus !== 'CONNECTED') {
      setStatus('Error: ROS not connected');
      return;
    }

    if (needsCoordinates || needsMultipleCoordinates) {
      setStatus(`Triggering ${actionName} via Proxy...`);
      
      const cmdTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/web/action_command',
        messageType: 'std_msgs/msg/String'
      });
      
      const payload = { action: actionName };
      
      if (needsCoordinates) {
        payload.x = parseFloat(x) || 0;
        payload.y = parseFloat(y) || 0;
        payload.yaw = parseFloat(yaw) || 0;
      } else if (needsMultipleCoordinates) {
        payload.waypoints = waypoints.map(wp => ({
          x: parseFloat(wp.x) || 0,
          y: parseFloat(wp.y) || 0,
          yaw: parseFloat(wp.yaw) || 0
        }));
      }
      
      cmdTopic.publish({ data: JSON.stringify(payload) });
      setStatus('Command sent via JSON Proxy!');
      return;
    }

    if (proxyService) {
      setStatus('Triggering via Proxy...');
      const service = new ROSLIB.Service({
        ros: ros,
        name: proxyService,
        serviceType: 'std_srvs/srv/Empty'
      });
      service.callService({}, 
        (res) => setStatus('Sent successfully!'),
        (err) => setStatus('Error: Proxy failed')
      );
      return;
    }

    console.log(`Triggering action: ${actionName} from UI (Requires Python Proxy Node for ROS 2)`);
    setStatus('No Proxy Configured');
  };

  const updateWaypoint = (index, field, value) => {
    const newWp = [...waypoints];
    newWp[index][field] = value;
    setWaypoints(newWp);
  };

  const addWaypoint = () => setWaypoints([...waypoints, {x: 0, y: 0, yaw: 0}]);
  const removeWaypoint = (index) => setWaypoints(waypoints.filter((_, i) => i !== index));

  const handleCancel = () => {
    if (!ros || connectionStatus !== 'CONNECTED') {
      setStatus('Error: ROS not connected');
      return;
    }

    const cancelService = new ROSLIB.Service({
      ros: ros,
      name: `${actionName}/_action/cancel_goal`,
      serviceType: 'action_msgs/srv/CancelGoal'
    });
    cancelService.callService(
      { goal_info: { goal_id: { uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] }, stamp: { sec: 0, nanosec: 0 } } },
      () => { setStatus('Cancelled!'); setTimeout(() => setStatus(''), 3000); },
      (err) => { setStatus(`Cancel error: ${err}`); setTimeout(() => setStatus(''), 4000); }
    );
  };

  return (
    <div 
      className="service-row"
      onMouseEnter={() => onHover({ title, actionName, longDesc, behaviorUse })}
    >
      <div className="row-header">
        <h3 className="row-title">{title}</h3>
        <span className="row-topic">{actionName}</span>
      </div>

      {needsCoordinates && (
        <div style={{ display: 'flex', gap: '10px', marginTop: '12px' }}>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            X (metry)
            <input type="number" step="0.5" value={x} onChange={(e) => setX(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '70px', marginTop: '4px' }} />
          </label>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            Y (metry)
            <input type="number" step="0.5" value={y} onChange={(e) => setY(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '70px', marginTop: '4px' }} />
          </label>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            Kąt (° Yaw)
            <input type="number" step="15" value={yaw} onChange={(e) => setYaw(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '70px', marginTop: '4px' }} />
          </label>
        </div>
      )}

      {needsMultipleCoordinates && (
        <div style={{ marginTop: '12px', display: 'flex', flexDirection: 'column', gap: '8px' }}>
          {waypoints.map((wp, i) => (
            <div key={i} style={{ display: 'flex', gap: '10px', alignItems: 'flex-end' }}>
              <span style={{color: '#a3a3a3', fontSize: '0.8rem', paddingBottom: '8px'}}>Pt {i+1}</span>
              <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
                X <input type="number" step="0.5" value={wp.x} onChange={(e) => updateWaypoint(i, 'x', e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '60px', marginTop: '4px' }} />
              </label>
              <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
                Y <input type="number" step="0.5" value={wp.y} onChange={(e) => updateWaypoint(i, 'y', e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '60px', marginTop: '4px' }} />
              </label>
              <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
                ° Yaw <input type="number" step="15" value={wp.yaw} onChange={(e) => updateWaypoint(i, 'yaw', e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '60px', marginTop: '4px' }} />
              </label>
              {waypoints.length > 1 && (
                <button onClick={() => removeWaypoint(i)} style={{background:'transparent', border:'none', color:'#f43f5e', cursor:'pointer', padding:'4px 0 8px 0'}}><XCircle size={18}/></button>
              )}
            </div>
          ))}
          <button onClick={addWaypoint} style={{ background: 'rgba(255,255,255,0.05)', border: '1px dashed #444', color: '#ccc', padding: '6px', borderRadius: '4px', width: 'fit-content', fontSize: '0.8rem', cursor: 'pointer', marginTop: '4px' }}>
            + Add Point
          </button>
        </div>
      )}
      
      <div style={{ display: 'flex', alignItems: 'center', gap: '16px', marginTop: '12px' }}>
        <button 
          className="orange-btn"
          onClick={handleFireAction}
        >
          {(proxyService || needsCoordinates || needsMultipleCoordinates) ? 'EXECUTE ACTION' : 'PREPARE ACTION'}
        </button>
        <button
          onClick={handleCancel}
          style={{
            background: 'rgba(244, 63, 94, 0.15)',
            border: '1px solid rgba(244, 63, 94, 0.4)',
            borderRadius: '6px',
            padding: '8px 14px',
            color: '#f43f5e',
            fontWeight: 700,
            fontSize: '0.78rem',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            gap: '6px',
            transition: 'all 0.2s ease',
            textTransform: 'uppercase',
            letterSpacing: '0.04em',
          }}
          onMouseEnter={(e) => { e.currentTarget.style.background = 'rgba(244, 63, 94, 0.3)'; }}
          onMouseLeave={(e) => { e.currentTarget.style.background = 'rgba(244, 63, 94, 0.15)'; }}
        >
          <XCircle size={14} />
          CANCEL
        </button>
        {status && <span style={{ fontSize: '0.8rem', color: '#f97316' }}>{status}</span>}
      </div>
    </div>
  );
};

export default function NavigationPage() {
  const [activeInfo, setActiveInfo] = useState(null);
  const [feedback, setFeedback] = useState(null);
  const { ros } = useRos();

  React.useEffect(() => {
    if (!ros) return;

    const feedbackTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/web/action_status',
      messageType: 'std_msgs/msg/String'
    });

    feedbackTopic.subscribe((message) => {
      try {
        const data = JSON.parse(message.data);
        setFeedback(data);
      } catch (e) {
        console.error("Error parsing feedback:", e);
      }
    });

    return () => feedbackTopic.unsubscribe();
  }, [ros]);

  return (
    <div className="map-page-wrapper">
      
      {/* LEFT COLUMN: The Vertical List */}
      <div className="map-list-section">
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '12px' }}>
          <div>
            <h1 style={{ margin: '0 0 4px 0', fontSize: '2rem', color: '#fff' }}>Nav2 Actions Control</h1>
            <p style={{ margin: 0, color: '#a3a3a3' }}>Hover over any action server to view details.</p>
          </div>
        </div>

        {/* Global Cancel All */}
        <div style={{ marginBottom: '16px' }}>
          <button
            onClick={() => {
              const cancelServices = [
                '/navigate_to_pose/_action/cancel_goal',
                '/navigate_through_poses/_action/cancel_goal',
                '/follow_waypoints/_action/cancel_goal',
                '/navigation/backup/_action/cancel_goal',
                '/navigation/spin/_action/cancel_goal',
                '/navigation/wait/_action/cancel_goal',
                '/navigation/drive_on_heading/_action/cancel_goal',
                '/navigation/follow_path/_action/cancel_goal',
              ];
              const cancelReq = {
                goal_info: {
                  goal_id: { uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] },
                  stamp: { sec: 0, nanosec: 0 }
                }
              };
              cancelServices.forEach(svcName => {
                if (ros) {
                  const svc = new ROSLIB.Service({ ros, name: svcName, serviceType: 'action_msgs/srv/CancelGoal' });
                  svc.callService(cancelReq, () => {}, () => {});
                }
              });
            }}
            style={{
              width: '100%',
              background: 'linear-gradient(135deg, #f43f5e, #e11d48)',
              border: 'none',
              borderRadius: '8px',
              padding: '14px 20px',
              color: '#fff',
              fontWeight: 700,
              fontSize: '1rem',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              gap: '10px',
              transition: 'all 0.2s ease',
              letterSpacing: '0.03em',
            }}
            onMouseEnter={(e) => { e.currentTarget.style.filter = 'brightness(1.15)'; }}
            onMouseLeave={(e) => { e.currentTarget.style.filter = 'brightness(1)'; }}
          >
            <XCircle size={20} />
            CANCEL ALL ACTIONS
          </button>
        </div>

        <CollapsibleCategory 
          title="Core Navigation Missions" 
          subtitle="Use these to command the rover to move autonomously to desired locations."
        >
          <ActionRow 
            title="Navigate to Pose" 
            actionName="/navigate_to_pose" 
            needsCoordinates={true}
            longDesc="Navigates the rover from its current position to a single target coordinate while avoiding dynamic obstacles. Does not provide live progress here because it uses direct topics."
            behaviorUse="Trigger this for standard A-to-B movement (e.g., 'go to the charging station')."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Navigate Through Poses" 
            actionName="/navigate_through_poses" 
            needsMultipleCoordinates={true}
            longDesc="Navigates through a list of points smoothly, treating them as intermediate checkpoints without stopping at each one."
            behaviorUse="Trigger when the rover needs to follow a specific route or patrol a curved corridor without slowing down."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Follow Waypoints" 
            actionName="/follow_waypoints" 
            needsMultipleCoordinates={true}
            longDesc="Navigates to a list of points, but strictly stops at each point to execute a task (via waypoint task executors)."
            behaviorUse="Trigger when the rover needs to inspect multiple machines, stopping at each one to take a photo."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Follow GPS Waypoints" 
            actionName="/navigation/follow_gps_waypoints" 
            longDesc="Same as follow_waypoints, but accepts latitude and longitude coordinates instead of map X/Y."
            behaviorUse="Trigger for outdoor missions relying on GPS rather than SLAM."
            onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Path Planning & Processing" 
          subtitle="Use these to calculate or modify trajectories without physically moving the rover."
        >
          <ActionRow 
            title="Compute Path To Pose" 
            actionName="/navigation/compute_path_to_pose" 
            needsCoordinates={true}
            longDesc="Asks the Global Planner to calculate a route to a destination and return the path, but does NOT move the rover."
            behaviorUse="Trigger this to preview the route on the UI to let the user approve it before actually driving."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Compute Path Through Poses" 
            actionName="/navigation/compute_path_through_poses" 
            needsMultipleCoordinates={true}
            longDesc="Calculates the complete route passing through multiple checkpoints without executing it."
            behaviorUse="Preview full patrol routes or multi-point inspections on the map before moving."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Smooth Path" 
            actionName="/navigation/smooth_path" 
            longDesc="Takes a rough, jagged path (often produced by standard grid planners) and mathematically smooths it."
            behaviorUse="Trigger this internally to optimize a path for a non-holonomic vehicle (like an Ackermann rover) to ensure it can take the turns smoothly."
            onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Low-Level Control" 
          subtitle="Use these to command the local controller directly."
        >
          <ActionRow 
            title="Follow Path" 
            actionName="/navigation/follow_path" 
            longDesc="Bypasses the Global Planner. You feed it a pre-calculated path, and the Local Controller strictly tries to follow it."
            behaviorUse="Trigger this if you generated a custom path externally (e.g., via your own algorithm) and just want the Nav2 controller to execute it."
            onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Recovery Behaviors (Panic & Unstuck)" 
          subtitle="Use these when the rover is stuck, surrounded by obstacles, or needs to reset its immediate environment."
        >
          <ActionRow 
            title="Backup" 
            actionName="/navigation/backup" 
            proxyService="/web/backup_proxy"
            longDesc="Commands the rover to drive straight backward for a specified distance, using collision checking."
            behaviorUse="Trigger when the rover drives too close to a wall and the local planner cannot find a way to turn around."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Spin" 
            actionName="/navigation/spin" 
            proxyService="/web/spin_proxy"
            longDesc="Commands the rover to rotate in place by a specified angle (e.g., 360 degrees)."
            behaviorUse="Trigger when localization is lost or costmaps are filled with ghosts; spinning allows the LiDAR/Camera to scan the whole room and update the map."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Wait" 
            actionName="/navigation/wait" 
            proxyService="/web/wait_proxy"
            longDesc="Commands the rover to pause in place and do nothing for a specified amount of time (default: 5 seconds)."
            behaviorUse="Trigger when a dynamic obstacle (like a human or another robot) blocks a narrow corridor, giving it time to move away before recalculating the path."
            onHover={setActiveInfo}
          />
          <ActionRow 
            title="Drive on Heading" 
            actionName="/navigation/drive_on_heading" 
            proxyService="/web/drive_proxy"
            longDesc="Drives the rover exactly straight on its current heading for a specified distance or time."
            behaviorUse="Trigger to escape tight corners or back out of narrow passages blindly when sensors are blocked or confused."
            onHover={setActiveInfo}
          />
        </CollapsibleCategory>
        
      </div>

      {/* RIGHT COLUMN: The Sticky Details Panel */}
      <div className="map-info-section">
        {activeInfo ? (
          <div style={{ animation: 'fadeIn 0.3s ease' }}>
            <h2 className="info-title">{activeInfo.title}</h2>
            <div className="info-topic">{activeInfo.actionName}</div>
            
            <p className="info-desc">{activeInfo.longDesc}</p>
            
            <div className="info-behavior">
              <strong>Behavior use:</strong><br/><br/>
              {activeInfo.behaviorUse}
            </div>

            {feedback && feedback.action === activeInfo.title && (
              <div style={{
                marginTop: '24px',
                padding: '16px',
                background: 'rgba(0,0,0,0.3)',
                borderRadius: '12px',
                border: `1px solid ${
                  feedback.status === 'completed' ? 'var(--status-emerald)' : 
                  feedback.status === 'error' ? 'var(--status-rose)' : 'var(--accent-cyan)'
                }44`
              }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '8px' }}>
                  <strong style={{ color: '#fff' }}>Live Status</strong>
                  <span style={{ 
                    color: feedback.status === 'completed' ? 'var(--status-emerald)' : 
                           feedback.status === 'error' ? 'var(--status-rose)' : 'var(--accent-cyan)',
                    textTransform: 'uppercase',
                    fontSize: '0.8rem',
                    fontWeight: 'bold'
                  }}>{feedback.status}</span>
                </div>
                
                {['starting', 'running'].includes(feedback.status) && feedback.target > 0 && (
                  <>
                    <div style={{ background: 'rgba(255,255,255,0.1)', height: '6px', borderRadius: '3px', overflow: 'hidden', marginTop: '12px' }}>
                      <div style={{ 
                        background: 'var(--accent-cyan)', 
                        height: '100%', 
                        width: `${Math.min(100, (feedback.traveled / feedback.target) * 100)}%`,
                        transition: 'width 0.2s linear'
                      }}></div>
                    </div>
                    <div style={{ display: 'flex', justifyContent: 'space-between', marginTop: '6px', fontSize: '0.8rem', color: '#a3a3a3' }}>
                      <span>{feedback.traveled.toFixed(2)} {activeInfo.title === 'Spin' ? 'rad' : activeInfo.title === 'Wait' ? 's' : 'm'}</span>
                      <span>{feedback.target.toFixed(2)} {activeInfo.title === 'Spin' ? 'rad' : activeInfo.title === 'Wait' ? 's' : 'm'}</span>
                    </div>
                  </>
                )}

                {['starting', 'running'].includes(feedback.status) && feedback.target < 0 && (
                  <div style={{ display: 'flex', justifyContent: 'space-between', marginTop: '12px', fontSize: '0.85rem', color: '#a3a3a3' }}>
                    <span>Distance remaining to goal:</span>
                    <strong style={{ color: 'var(--accent-cyan)' }}>{feedback.traveled.toFixed(2)} m</strong>
                  </div>
                )}
              </div>
            )}
          </div>
        ) : (
          <div className="empty-info">
            <Info size={48} style={{ color: '#ea580c', margin: '0 auto 16px auto', display: 'block', opacity: 0.5 }} />
            <h3 style={{ color: '#fff', textAlign: 'center' }}>Awaiting Selection</h3>
            <p style={{ textAlign: 'center' }}>Hover over any action on the left to view detailed descriptions and behavioral use cases.</p>
          </div>
        )}
      </div>

    </div>
  );
}
