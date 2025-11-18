import { useCallback } from 'react';
import type { JointInfo } from '../types';
import './JointControlPanel.css';

interface JointControlPanelProps {
  joints: JointInfo[];
  onJointChange: (index: number, value: number) => void;
  disabled?: boolean;
}

export function JointControlPanel({ joints, onJointChange, disabled = false }: JointControlPanelProps) {
  const handleSliderChange = useCallback(
    (index: number, event: React.ChangeEvent<HTMLInputElement>) => {
      const value = parseFloat(event.target.value);
      onJointChange(index, value);
    },
    [onJointChange]
  );

  const formatAngle = (radians: number): string => {
    const degrees = (radians * 180) / Math.PI;
    return `${degrees.toFixed(1)}°`;
  };

  if (joints.length === 0) {
    return (
      <div className="joint-control-panel">
        <h3>Joint Controls</h3>
        <p className="no-joints">No joints available</p>
      </div>
    );
  }

  return (
    <div className="joint-control-panel">
      <h3>Joint Controls</h3>
      <div className="joints-list">
        {joints.map((joint, index) => (
          <div key={joint.name} className="joint-control">
            <div className="joint-header">
              <label htmlFor={`joint-${index}`}>{joint.name}</label>
              <span className="joint-value">{formatAngle(joint.value)}</span>
            </div>
            <div className="joint-slider-container">
              <input
                id={`joint-${index}`}
                type="range"
                min={joint.lowerLimit}
                max={joint.upperLimit}
                step={0.01}
                value={joint.value}
                onChange={(e) => handleSliderChange(index, e)}
                disabled={disabled}
                className="joint-slider"
              />
              <div className="joint-limits">
                <span>{formatAngle(joint.lowerLimit)}</span>
                <span>{formatAngle(joint.upperLimit)}</span>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
