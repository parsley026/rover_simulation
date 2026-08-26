import React, { useState } from 'react';
import { useRos } from '../context/RosContext';
import * as ROSLIB from 'roslib';
import { Info, ChevronDown } from 'lucide-react';
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

const ServiceRow = ({ title, shortDesc, longDesc, behaviorUse, serviceName, serviceType = 'std_srvs/srv/Empty', fields = [], onResult, onHover }) => {
  const { ros, connectionStatus } = useRos();
  const [loading, setLoading] = useState(false);
  const [inputValues, setInputValues] = useState({});

  const handleInputChange = (field, value) => {
    setInputValues(prev => ({ ...prev, [field]: value }));
  };

  const handleCall = () => {
    if (!ros) {
      onResult(`Error: ROS is not connected (${connectionStatus})`);
      return;
    }

    setLoading(true);

    const service = new ROSLIB.Service({
      ros: ros,
      name: serviceName,
      serviceType: serviceType
    });

    const requestData = {};
    fields.forEach(f => {
      let val = inputValues[f.name];
      if (f.type === 'bool') val = !!val;
      if (f.type === 'int32') val = parseInt(val || 0, 10);
      requestData[f.name] = val !== undefined ? val : '';
    });

    service.callService(
      requestData,
      (result) => {
        setLoading(false);
        console.log(`[${serviceName}] success:`, result);
        if (onResult) onResult(`Success: ${JSON.stringify(result)}`);
      },
      (error) => {
        setLoading(false);
        console.error(`[${serviceName}] failed:`, error);
        if (onResult) onResult(`Failed: ${error}`);
      }
    );
  };

  return (
    <div 
      className="service-row"
      onMouseEnter={() => onHover({ title, serviceName, longDesc, behaviorUse })}
    >
      <div className="row-header">
        <h3 className="row-title">{title}</h3>
        <span className="row-topic">{serviceName}</span>
      </div>
      
      {fields.length > 0 && (
        <div className="row-inputs">
          {fields.map(f => (
            <div key={f.name} style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <label style={{ fontSize: '0.8rem', color: '#a3a3a3', minWidth: '150px' }}>{f.label}</label>
              <input 
                className="orange-input"
                type={f.type === 'bool' ? 'checkbox' : 'text'}
                placeholder={f.placeholder}
                value={inputValues[f.name] || ''}
                onChange={(e) => handleInputChange(f.name, f.type === 'bool' ? e.target.checked : e.target.value)}
              />
            </div>
          ))}
        </div>
      )}

      <button 
        className="orange-btn"
        disabled={loading || connectionStatus !== 'CONNECTED'}
        onClick={handleCall}
      >
        {loading ? 'WAIT...' : 'EXECUTE'}
      </button>
    </div>
  );
};

export default function MapPage() {
  const [lastResult, setLastResult] = useState('');
  const [activeInfo, setActiveInfo] = useState(null);

  const showResult = (msg) => {
    setLastResult(`[${new Date().toLocaleTimeString()}] ${msg}`);
  };

  return (
    <div className="map-page-wrapper">
      
      {/* LEFT COLUMN: The Vertical List */}
      <div className="map-list-section">
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '12px' }}>
          <div>
            <h1 style={{ margin: '0 0 4px 0', fontSize: '2rem', color: '#fff' }}>RTAB-Map Control</h1>
            <p style={{ margin: 0, color: '#a3a3a3' }}>Hover over any module to view details.</p>
          </div>
          {lastResult && (
            <div style={{ background: '#000', border: '1px solid #ea580c', color: '#ea580c', padding: '8px 12px', borderRadius: '6px', fontSize: '0.8rem' }}>
              {lastResult}
            </div>
          )}
        </div>

        <CollapsibleCategory 
          title="Core State Control" 
          subtitle="Use these to toggle how the rover interacts with its environment on the fly."
        >
          <ServiceRow 
            title="Pause SLAM" 
            serviceName="/mapping/rtabmap_slam/pause" 
            longDesc="Freezes SLAM processing."
            behaviorUse="Trigger pause when driving through featuresless terrain (like a flat field) where SLAM will produce garbage."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Resume SLAM" 
            serviceName="/mapping/rtabmap_slam/resume" 
            longDesc="Unfreezes SLAM processing."
            behaviorUse="Trigger resume when structured geometry returns."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Set Mode Localization" 
            serviceName="/mapping/rtabmap_slam/set_mode_localization" 
            longDesc="Stops adding new data to the map (read-only mode)."
            behaviorUse="Trigger this automatically once the rover has fully mapped a room to prevent dynamic objects (people, moving boxes) from being permanently burned into the map."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Set Mode Mapping" 
            serviceName="/mapping/rtabmap_slam/set_mode_mapping" 
            longDesc="Resumes active map building (read-write mode)."
            behaviorUse="Trigger this to actively learn new environments or update changes in geometry."
            onResult={showResult} onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Map Recovery & Synchronization" 
          subtitle="Use these to fix the map when the rover gets confused or sensors desync."
        >
          <ServiceRow 
            title="Cleanup Local Grids" 
            serviceName="/mapping/rtabmap_slam/cleanup_local_grids" 
            longDesc="Erases temporary noise from the immediate area."
            behaviorUse="Trigger when the local planner gets stuck surrounded by 'ghost' obstacles that are no longer there."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Reset Map" 
            serviceName="/mapping/rtabmap_slam/reset" 
            longDesc="Wipes the current map in memory and starts completely fresh."
            behaviorUse="The ultimate 'panic button' behavior if the map is completely corrupted and localization is lost."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Trigger New Map" 
            serviceName="/mapping/rtabmap_slam/trigger_new_map" 
            longDesc="Starts a new map session but keeps the previous one safely in the database."
            behaviorUse="Trigger when the rover transitions to a drastically new area (e.g., driving from indoors to outdoors)."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Publish Map" 
            serviceName="/mapping/rtabmap_slam/publish_map" 
            longDesc="Forces RTAB-Map to broadcast its latest map to Nav2 immediately."
            behaviorUse="Trigger if the SLAM map and Nav2 costmaps fall out of sync and the rover is acting 'blind'."
            onResult={showResult} onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Graph Optimization" 
          subtitle="Use these to dedicate compute power to fixing map drift."
        >
          <ServiceRow 
            title="Global Bundle Adjustment" 
            serviceName="/mapping/rtabmap_slam/global_bundle_adjustment" 
            longDesc="Triggers a heavy, full-map alignment."
            behaviorUse="Call this when the rover docks or is stationary to clean up all accumulated map drift while it has spare CPU."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Detect More Loop Closures" 
            serviceName="/mapping/rtabmap_slam/detect_more_loop_closures" 
            longDesc="Forces the system to search harder for familiar visual/LiDAR features."
            behaviorUse="Trigger if localization confidence drops; have the rover spin 360 degrees and run this service to figure out where it is."
            onResult={showResult} onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Map Persistence" 
          subtitle="Use these for saving and restoring states across missions or reboots."
        >
          <ServiceRow 
            title="Backup Database" 
            serviceName="/mapping/rtabmap_slam/backup" 
            longDesc="Saves the current uncorrupted database state to disk."
            behaviorUse="Trigger an auto-save right before the rover attempts a difficult or risky obstacle traversal."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Load Database" 
            serviceName="/mapping/rtabmap_slam/load_database" serviceType="rtabmap_msgs/srv/LoadDatabase"
            longDesc="Injects a previously saved .db map file."
            behaviorUse="Trigger upon bootup in a known environment so the rover doesn't have to remap the area."
            fields={[
              { name: 'database_path', label: 'Database Path (.db)', type: 'string', placeholder: '/home/maurycy/maps/rtabmap.db' },
              { name: 'clear', label: 'Clear Database', type: 'bool' }
            ]}
            onResult={showResult} onHover={setActiveInfo}
          />
        </CollapsibleCategory>

        <CollapsibleCategory 
          title="Topological Navigation (Semantic Memory)" 
          subtitle="Use these to navigate by concept rather than XY coordinates, which is highly resistant to map drift."
        >
          <ServiceRow 
            title="Set Label" 
            serviceName="/mapping/rtabmap_slam/set_label" serviceType="rtabmap_msgs/srv/SetLabel"
            longDesc="Tags the current graph node with a semantic name (e.g., 'sample_1', 'charging_dock')."
            behaviorUse="Have the autonomy system automatically tag the location whenever it finds an object of interest."
            fields={[
              { name: 'node_label', label: 'Semantic Name', type: 'string', placeholder: 'e.g., charging_dock' },
              { name: 'node_id', label: 'Node ID (0 for current)', type: 'int32', placeholder: '0' }
            ]}
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="List Labels" 
            serviceName="/mapping/rtabmap_slam/list_labels" serviceType="rtabmap_msgs/srv/ListLabels"
            longDesc="Retrieves all semantic tags in the current map."
            behaviorUse="Check all landmarks known by the system."
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Set Goal" 
            serviceName="/mapping/rtabmap_slam/set_goal" serviceType="rtabmap_msgs/srv/SetGoal"
            longDesc="Commands RTAB-Map to generate a path to a specific label (e.g., 'charging_dock') rather than a coordinate."
            behaviorUse="Navigate to places using abstract semantic concepts, avoiding absolute XYZ coordinate shifts."
            fields={[
              { name: 'node_label', label: 'Goal Label', type: 'string', placeholder: 'e.g., charging_dock' },
              { name: 'node_id', label: 'Goal Node ID', type: 'int32', placeholder: '0' },
              { name: 'frame_id', label: 'Frame ID', type: 'string', placeholder: 'map' }
            ]}
            onResult={showResult} onHover={setActiveInfo}
          />
          <ServiceRow 
            title="Cancel Goal" 
            serviceName="/mapping/rtabmap_slam/cancel_goal" 
            longDesc="Aborts the current topological path."
            behaviorUse="Cancel a running topological mission manually."
            onResult={showResult} onHover={setActiveInfo}
          />
        </CollapsibleCategory>
        
      </div>

      {/* RIGHT COLUMN: The Sticky Details Panel */}
      <div className="map-info-section">
        {activeInfo ? (
          <div style={{ animation: 'fadeIn 0.3s ease' }}>
            <h2 className="info-title">{activeInfo.title}</h2>
            <div className="info-topic">{activeInfo.serviceName}</div>
            
            <p className="info-desc">{activeInfo.longDesc}</p>
            
            <div className="info-behavior">
              <strong>Behavior use:</strong><br/><br/>
              {activeInfo.behaviorUse}
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
