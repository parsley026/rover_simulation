import React, { useState } from 'react';
import { Settings, Navigation } from 'lucide-react';
import Nav2Control from '../components/Nav2Control';

const TABS = [
  {
    id: 'nav2',
    label: 'Nav2 Control',
    icon: Navigation,
    component: <Nav2Control />,
  },
  // Kolejne zakładki można dodać tutaj
  // { id: 'params', label: 'ROS Params', icon: Settings, component: <RosParams /> },
];

export default function ConfigPage() {
  const [activeTab, setActiveTab] = useState(TABS[0].id);

  const active = TABS.find((t) => t.id === activeTab);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, overflow: 'hidden', padding: '16px', gap: '0' }}>
      {/* Page Header */}
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: '12px',
        marginBottom: '20px',
        paddingBottom: '16px',
        borderBottom: '1px solid rgba(255,255,255,0.08)',
      }}>
        <Settings size={22} style={{ color: 'var(--accent-cyan)' }} />
        <div>
          <h1 style={{ margin: 0, fontSize: '1.2rem', fontWeight: 700 }}>Configuration</h1>
          <p style={{ margin: 0, fontSize: '0.82rem', color: 'var(--text-muted)' }}>Ustawienia łazika i nawigacji Nav2</p>
        </div>
      </div>

      {/* Tab Bar */}
      <div style={{
        display: 'flex',
        gap: '4px',
        marginBottom: '0',
        background: 'rgba(255,255,255,0.03)',
        border: '1px solid rgba(255,255,255,0.08)',
        borderBottom: 'none',
        borderRadius: '10px 10px 0 0',
        padding: '6px 6px 0 6px',
      }}>
        {TABS.map((tab) => {
          const Icon = tab.icon;
          const isActive = tab.id === activeTab;
          return (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: '8px',
                padding: '9px 18px',
                border: 'none',
                borderRadius: '7px 7px 0 0',
                cursor: 'pointer',
                fontFamily: 'var(--font-sans)',
                fontWeight: isActive ? 600 : 400,
                fontSize: '0.9rem',
                color: isActive ? 'var(--accent-cyan)' : 'var(--text-muted)',
                background: isActive
                  ? 'rgba(0,242,254,0.08)'
                  : 'transparent',
                borderBottom: isActive ? '2px solid var(--accent-cyan)' : '2px solid transparent',
                transition: 'all 0.2s ease',
              }}
              onMouseEnter={(e) => { if (!isActive) e.currentTarget.style.color = 'var(--text-main)'; }}
              onMouseLeave={(e) => { if (!isActive) e.currentTarget.style.color = 'var(--text-muted)'; }}
            >
              <Icon size={16} />
              {tab.label}
            </button>
          );
        })}
      </div>

      {/* Tab Content Panel */}
      <div
        className="glass-panel"
        style={{
          flex: 1,
          overflowY: 'auto',
          borderRadius: '0 10px 10px 10px',
          display: 'flex',
          flexDirection: 'column',
        }}
      >
        {active?.component}
      </div>
    </div>
  );
}
