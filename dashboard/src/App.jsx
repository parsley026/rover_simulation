import React from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { RosProvider } from './context/RosContext';
import Navbar from './components/Navbar';
import ChatPanel from './components/ChatPanel';
import TelemetryInspector from './components/TelemetryInspector';
import ConfigPage from './pages/ConfigPage';
import './index.css';

function DashboardHome() {
  return (
    <main className="main-content">
      <ChatPanel />
      <TelemetryInspector />
    </main>
  );
}

export default function App() {
  return (
    <RosProvider url="ws://localhost:9090">
      <BrowserRouter>
        <div className="app-container">
          <Navbar />
          <Routes>
            <Route path="/" element={<DashboardHome />} />
            <Route path="/config" element={<ConfigPage />} />
            <Route path="*" element={<Navigate to="/" replace />} />
          </Routes>
        </div>
      </BrowserRouter>
    </RosProvider>
  );
}
