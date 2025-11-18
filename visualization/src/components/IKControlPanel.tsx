import { useState } from 'react';
import './IKControlPanel.css';

interface IKControlPanelProps {
  targetPose: { position: [number, number, number]; quaternion: [number, number, number, number] };
  onTargetChange: (pose: { position: [number, number, number]; quaternion: [number, number, number, number] }) => void;
  onSolve: () => void;
  disabled?: boolean;
  solutionFound?: boolean;
}

export function IKControlPanel({ 
  targetPose, 
  onTargetChange, 
  onSolve,
  disabled = false,
  solutionFound
}: IKControlPanelProps) {
  const [editMode, setEditMode] = useState<'position' | 'orientation'>('position');

  const handlePositionChange = (axis: 0 | 1 | 2, value: number) => {
    const newPosition = [...targetPose.position] as [number, number, number];
    newPosition[axis] = value;
    onTargetChange({ ...targetPose, position: newPosition });
  };

  const handleOrientationChange = (index: 0 | 1 | 2 | 3, value: number) => {
    const newQuaternion = [...targetPose.quaternion] as [number, number, number, number];
    newQuaternion[index] = value;
    
    // Normalize quaternion
    const norm = Math.sqrt(
      newQuaternion[0] ** 2 + 
      newQuaternion[1] ** 2 + 
      newQuaternion[2] ** 2 + 
      newQuaternion[3] ** 2
    );
    
    if (norm > 0.001) {
      newQuaternion[0] /= norm;
      newQuaternion[1] /= norm;
      newQuaternion[2] /= norm;
      newQuaternion[3] /= norm;
    }
    
    onTargetChange({ ...targetPose, quaternion: newQuaternion });
  };

  return (
    <div className="ik-control-panel">
      <h3>Inverse Kinematics</h3>
      
      <div className="ik-hint">
        <p>🎯 在 3D 视图中拖拽红色球体来移动目标位置</p>
      </div>

      <div className="edit-mode-toggle">
        <button
          className={editMode === 'position' ? 'active' : ''}
          onClick={() => setEditMode('position')}
          disabled={disabled}
        >
          位置
        </button>
        <button
          className={editMode === 'orientation' ? 'active' : ''}
          onClick={() => setEditMode('orientation')}
          disabled={disabled}
        >
          姿态
        </button>
      </div>

      {editMode === 'position' && (
        <div className="position-controls">
          <div className="control-group">
            <label>X (m)</label>
            <input
              type="number"
              value={targetPose.position[0].toFixed(4)}
              onChange={(e) => handlePositionChange(0, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
            <input
              type="range"
              value={targetPose.position[0]}
              onChange={(e) => handlePositionChange(0, parseFloat(e.target.value))}
              min="-2"
              max="2"
              step="0.01"
              disabled={disabled}
            />
          </div>

          <div className="control-group">
            <label>Y (m)</label>
            <input
              type="number"
              value={targetPose.position[1].toFixed(4)}
              onChange={(e) => handlePositionChange(1, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
            <input
              type="range"
              value={targetPose.position[1]}
              onChange={(e) => handlePositionChange(1, parseFloat(e.target.value))}
              min="-2"
              max="2"
              step="0.01"
              disabled={disabled}
            />
          </div>

          <div className="control-group">
            <label>Z (m)</label>
            <input
              type="number"
              value={targetPose.position[2].toFixed(4)}
              onChange={(e) => handlePositionChange(2, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
            <input
              type="range"
              value={targetPose.position[2]}
              onChange={(e) => handlePositionChange(2, parseFloat(e.target.value))}
              min="-2"
              max="2"
              step="0.01"
              disabled={disabled}
            />
          </div>
        </div>
      )}

      {editMode === 'orientation' && (
        <div className="orientation-controls">
          <div className="control-group">
            <label>qw</label>
            <input
              type="number"
              value={targetPose.quaternion[0].toFixed(4)}
              onChange={(e) => handleOrientationChange(0, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
          </div>

          <div className="control-group">
            <label>qx</label>
            <input
              type="number"
              value={targetPose.quaternion[1].toFixed(4)}
              onChange={(e) => handleOrientationChange(1, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
          </div>

          <div className="control-group">
            <label>qy</label>
            <input
              type="number"
              value={targetPose.quaternion[2].toFixed(4)}
              onChange={(e) => handleOrientationChange(2, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
          </div>

          <div className="control-group">
            <label>qz</label>
            <input
              type="number"
              value={targetPose.quaternion[3].toFixed(4)}
              onChange={(e) => handleOrientationChange(3, parseFloat(e.target.value))}
              step="0.01"
              disabled={disabled}
            />
          </div>
        </div>
      )}

      <button 
        className="solve-button"
        onClick={onSolve}
        disabled={disabled}
      >
        求解 IK
      </button>

      {solutionFound !== undefined && (
        <div className={`solution-status ${solutionFound ? 'success' : 'failure'}`}>
          {solutionFound ? '✓ 找到解' : '✗ 未找到解'}
        </div>
      )}
    </div>
  );
}
