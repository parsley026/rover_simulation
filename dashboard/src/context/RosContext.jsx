import React, { createContext, useContext, useEffect, useRef, useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';

const RosContext = createContext();

export const useRos = () => {
  const context = useContext(RosContext);
  if (!context) {
    throw new Error('useRos must be used within a RosProvider');
  }
  return context;
};

export const RosProvider = ({ children, url = 'ws://localhost:9090' }) => {
  const rosRef = useRef(null);
  const isInitializedRef = useRef(false);
  const reconnectTimeoutRef = useRef(null);
  const rosTimeRef = useRef({ sec: 0, nanosec: 0 });

  const [connectionStatus, setConnectionStatus] = useState('CONNECTING'); // 'CONNECTED', 'CLOSED', 'ERROR', 'CONNECTING'
  const [lastError, setLastError] = useState(null);

  // Maintain references to active topics to prevent redundant topic objects
  const topicsRef = useRef({});

  const connectToRos = useCallback(() => {
    if (rosRef.current && rosRef.current.isConnected) {
      return;
    }

    try {
      setConnectionStatus('CONNECTING');
      const ros = new ROSLIB.Ros({ url });

      ros.on('connection', () => {
        console.log('[RosContext] WebSocket connected to gateway at:', url);
        setConnectionStatus('CONNECTED');
        setLastError(null);

        // Subscribe to /clock to track simulation time for message headers
        const clockTopic = new ROSLIB.Topic({
          ros,
          name: '/clock',
          messageType: 'rosgraph_msgs/Clock',
        });
        clockTopic.subscribe((msg) => {
          if (msg?.clock) {
            rosTimeRef.current = { sec: msg.clock.sec, nanosec: msg.clock.nanosec ?? 0 };
          }
        });
      });

      ros.on('error', (err) => {
        console.warn('[RosContext] WebSocket connection error:', err);
        setConnectionStatus('ERROR');
        setLastError('Connection error or gateway offline');
      });

      ros.on('close', () => {
        console.log('[RosContext] WebSocket closed. Attempting auto-reconnect in 3s...');
        setConnectionStatus('CLOSED');
        rosRef.current = null;
        if (!reconnectTimeoutRef.current) {
          reconnectTimeoutRef.current = setTimeout(() => {
            reconnectTimeoutRef.current = null;
            connectToRos();
          }, 3000);
        }
      });

      rosRef.current = ros;
    } catch (e) {
      console.error('[RosContext] Initialization error:', e);
      setConnectionStatus('ERROR');
    }
  }, [url]);

  useEffect(() => {
    // React Strict Mode Protection: Initialize ROSLIB singleton exactly once per mount cycle
    if (!isInitializedRef.current) {
      isInitializedRef.current = true;
      connectToRos();
    }

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      if (rosRef.current && rosRef.current.isConnected) {
        try {
          rosRef.current.close();
        } catch (e) {
          console.debug('Error closing ROS socket during teardown:', e);
        }
      }
      isInitializedRef.current = false;
      rosRef.current = null;
    };
  }, [connectToRos]);

  // Get or create a ROSLIB Topic singleton reference
  const getTopic = useCallback((name, messageType) => {
    if (!rosRef.current) return null;
    const key = `${name}_${messageType}`;
    if (!topicsRef.current[key]) {
      topicsRef.current[key] = new ROSLIB.Topic({
        ros: rosRef.current,
        name: name,
        messageType: messageType
      });
    }
    return topicsRef.current[key];
  }, []);

  // Publish helper method
  const publish = useCallback((topicName, messageType, messageData) => {
    const topic = getTopic(topicName, messageType);
    if (topic) {
      topic.publish(messageData);
      return true;
    }
    return false;
  }, [getTopic]);

  // Returns current ROS/sim time from /clock (falls back to wall time if clock not yet received)
  const getRosTime = useCallback(() => {
    if (rosTimeRef.current.sec > 0) return rosTimeRef.current;
    const now = Date.now();
    return { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 };
  }, []);

  const value = {
    ros: rosRef.current,
    connectionStatus,
    lastError,
    getTopic,
    publish,
    getRosTime,
    reconnect: connectToRos
  };

  return (
    <RosContext.Provider value={value}>
      {children}
    </RosContext.Provider>
  );
};
