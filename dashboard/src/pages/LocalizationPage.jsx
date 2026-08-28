import React, { useState } from 'react';
import { useRos } from '../context/RosContext';
import * as ROSLIB from 'roslib';
import { Info, ChevronDown } from 'lucide-react';
import './MapPage.css';

const CollapsibleCategory = ({ title, subtitle, children }) => {
  const [isOpen, setIsOpen] = useState(true);

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

const ServiceRow = ({ title, serviceName, serviceType = 'std_srvs/srv/Empty', needsCoordinates = false, desc, behavior, payloadType, onHover }) => {
  const { ros, connectionStatus } = useRos();
  const [status, setStatus] = useState('');
  
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [yaw, setYaw] = useState(0);

  const handleCall = () => {
    if (!ros || connectionStatus !== 'CONNECTED') {
      setStatus('Error: ROS not connected');
      return;
    }

    setStatus('Sending...');
    
    const service = new ROSLIB.Service({
      ros: ros,
      name: serviceName,
      serviceType: serviceType
    });

    let requestPayload = {};

    if (needsCoordinates) {
      const yawRad = parseFloat(yaw) * Math.PI / 180.0;
      if (payloadType === 'robot_localization') {
        requestPayload = {
          pose: {
            header: { frame_id: 'map' },
            pose: {
              pose: {
                position: { x: parseFloat(x) || 0.0, y: parseFloat(y) || 0.0, z: 0.0 },
                orientation: { x: 0.0, y: 0.0, z: Math.sin(yawRad / 2.0), w: Math.cos(yawRad / 2.0) }
              },
              covariance: [
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.068
              ]
            }
          }
        };
      } else if (payloadType === 'rtabmap') {
        requestPayload = {
          x: parseFloat(x) || 0.0,
          y: parseFloat(y) || 0.0,
          z: 0.0,
          roll: 0.0,
          pitch: 0.0,
          yaw: yawRad
        };
      }
    }

    service.callService(requestPayload, 
      (res) => setStatus('Success!'),
      (err) => setStatus('Execution error')
    );
  };

  return (
    <div 
      className="service-row"
      onMouseEnter={() => onHover({ title, topic: serviceName, desc, behavior })}
    >
      <div className="row-header">
        <h3 className="row-title">{title}</h3>
        <span className="row-topic">{serviceName}</span>
      </div>

      {needsCoordinates && (
        <div style={{ display: 'flex', gap: '10px', marginTop: '12px' }}>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            X (meters)
            <input type="number" step="0.5" value={x} onChange={(e) => setX(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '80px', marginTop: '4px' }} />
          </label>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            Y (meters)
            <input type="number" step="0.5" value={y} onChange={(e) => setY(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '80px', marginTop: '4px' }} />
          </label>
          <label style={{ display: 'flex', flexDirection: 'column', color: '#a3a3a3', fontSize: '0.8rem' }}>
            Angle (° Yaw)
            <input type="number" step="15" value={yaw} onChange={(e) => setYaw(e.target.value)} style={{ background: '#111', border: '1px solid #333', color: '#fff', padding: '4px 8px', borderRadius: '4px', width: '80px', marginTop: '4px' }} />
          </label>
        </div>
      )}
      
      <div style={{ display: 'flex', alignItems: 'center', gap: '16px', marginTop: '16px' }}>
        <button className="orange-btn" onClick={handleCall}>
          EXECUTE
        </button>
        {status && <span style={{ fontSize: '0.8rem', color: '#f97316' }}>{status}</span>}
      </div>
    </div>
  );
};

export default function LocalizationPage() {
  const [activeInfo, setActiveInfo] = useState(null);

  return (
    <div className="map-page-wrapper">
      <div className="map-list-section">
        
        <CollapsibleCategory 
          title="Localization Filtering (EKF)" 
          subtitle="Manage the core robot_localization filter"
        >
          <ServiceRow 
            title="Enable Localization" 
            serviceName="/localization/enable" 
            desc="Enables sensor input processing to the filter."
            behavior="Use after manually resetting algorithms."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Reset Filter" 
            serviceName="/localization/reset" 
            desc="Clears filter history and covariance, starting fresh."
            behavior="Use when wheel slip or odometry completely detunes localization."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Set Pose" 
            serviceName="/localization/set_pose"
            serviceType="robot_localization/srv/SetPose"
            needsCoordinates={true}
            payloadType="robot_localization"
            desc="Hard-forces the robot's current position in the EKF filter."
            behavior="Use as a fallback if the EKF gets completely lost."
            onHover={setActiveInfo} 
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Camera Odometry" 
          subtitle="Manage visual odometry from the RGB-D camera (/camera_00)"
        >
          <ServiceRow 
            title="Pause" 
            serviceName="/camera_00/pause_odom" 
            desc="Pauses visual odometry generation."
            behavior="Use when entering a crowd and the camera loses features."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Resume" 
            serviceName="/camera_00/resume_odom" 
            desc="Resumes visual odometry."
            behavior="Use after returning to a well-lit area."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Reset" 
            serviceName="/camera_00/reset_odom" 
            desc="Dumps internal visual odometry memory."
            behavior="Use when visual odometry drifts significantly."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Reset Odom to Pose" 
            serviceName="/camera_00/reset_odom_to_pose"
            serviceType="rtabmap_msgs/srv/ResetPose"
            needsCoordinates={true}
            payloadType="rtabmap"
            desc="Changes the origin frame of the camera odometry."
            behavior="Local reset for hard-setting the starting point."
            onHover={setActiveInfo} 
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="LiDAR Odometry" 
          subtitle="Manage laser odometry (/lidar_00)"
        >
          <ServiceRow 
            title="Pause" 
            serviceName="/lidar_00/pause_odom" 
            desc="Pauses laser odometry generation."
            behavior="Use when surrounded by glass walls or long featureless corridors."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Resume" 
            serviceName="/lidar_00/resume_odom" 
            desc="Resumes LiDAR odometry."
            behavior="Use after exiting a difficult environment."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Reset" 
            serviceName="/lidar_00/reset_odom" 
            desc="Dumps internal point cloud error."
            behavior="Use when the scan is too distorted to match."
            onHover={setActiveInfo} 
          />
          <ServiceRow 
            title="Reset Odom to Pose" 
            serviceName="/lidar_00/reset_odom_to_pose"
            serviceType="rtabmap_msgs/srv/ResetPose"
            needsCoordinates={true}
            payloadType="rtabmap"
            desc="Changes the zero point for laser measurements."
            behavior="Local reset of the scanner map."
            onHover={setActiveInfo} 
          />
        </CollapsibleCategory>
        
      </div>

      <div className="map-info-section">
        {activeInfo ? (
          <div style={{ animation: 'fadeIn 0.3s ease' }}>
            <h2 className="info-title">{activeInfo.title}</h2>
            <div className="info-topic">{activeInfo.topic}</div>
            
            <p className="info-desc">{activeInfo.desc}</p>
            
            <div className="info-behavior">
              <strong>Behavior use:</strong><br/><br/>
              {activeInfo.behavior}
            </div>
          </div>
        ) : (
          <div className="empty-info">
            <Info size={48} style={{ color: '#ea580c', marginBottom: '16px', opacity: 0.5 }} />
            <h3 style={{ color: '#fff' }}>Awaiting Selection</h3>
            <p>Hover over any module on the left to view detailed service parameters and behavioral use cases.</p>
          </div>
        )}
      </div>

    </div>
  );
}
