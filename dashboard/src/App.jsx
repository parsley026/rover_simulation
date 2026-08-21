import React from 'react';
import { RosProvider } from './context/RosContext';
import Navbar from './components/Navbar';
import ChatPanel from './components/ChatPanel';
import TelemetryInspector from './components/TelemetryInspector';
import './index.css';

export default function App() {
  return (
    <RosProvider url="ws://localhost:9090">
      <div className="app-container">
        <Navbar />
        <main className="main-content">
          <ChatPanel />
          <TelemetryInspector />
        </main>
      </div>
    </RosProvider>
  );
}
