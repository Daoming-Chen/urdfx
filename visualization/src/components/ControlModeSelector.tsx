import type { ControlMode } from '../types';
import './ControlModeSelector.css';

interface ControlModeSelectorProps {
  mode: ControlMode;
  onModeChange: (mode: ControlMode) => void;
  disabled?: boolean;
}

export function ControlModeSelector({ mode, onModeChange, disabled = false }: ControlModeSelectorProps) {
  return (
    <div className="control-mode-selector">
      <h3>Control Mode</h3>
      <div className="mode-buttons">
        <button
          className={`mode-button ${mode === 'FK' ? 'active' : ''}`}
          onClick={() => onModeChange('FK')}
          disabled={disabled}
        >
          Forward Kinematics
        </button>
        <button
          className={`mode-button ${mode === 'IK' ? 'active' : ''}`}
          onClick={() => onModeChange('IK')}
          disabled={disabled}
        >
          Inverse Kinematics
        </button>
      </div>
    </div>
  );
}
