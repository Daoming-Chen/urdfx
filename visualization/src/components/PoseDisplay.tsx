import type { Pose } from '../../public/urdfx';
import './PoseDisplay.css';

interface PoseDisplayProps {
  pose: Pose | null;
  label?: string;
}

export function PoseDisplay({ pose, label = 'End Effector Pose' }: PoseDisplayProps) {
  if (!pose) {
    return (
      <div className="pose-display">
        <h3>{label}</h3>
        <p className="no-pose">No pose data available</p>
      </div>
    );
  }

  const formatNumber = (num: number): string => num.toFixed(4);

  // Convert quaternion to Euler angles for display (approximate)
  const { quaternion } = pose;
  const [w, x, y, z] = quaternion;
  
  // Convert to Euler angles (ZYX convention)
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(2 * (w * y - z * x));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  const toDegrees = (rad: number) => ((rad * 180) / Math.PI).toFixed(1);

  return (
    <div className="pose-display">
      <h3>{label}</h3>
      
      <div className="pose-section">
        <h4>Position (m)</h4>
        <div className="pose-values">
          <div className="pose-value">
            <span className="label">X:</span>
            <span className="value">{formatNumber(pose.position[0])}</span>
          </div>
          <div className="pose-value">
            <span className="label">Y:</span>
            <span className="value">{formatNumber(pose.position[1])}</span>
          </div>
          <div className="pose-value">
            <span className="label">Z:</span>
            <span className="value">{formatNumber(pose.position[2])}</span>
          </div>
        </div>
      </div>

      <div className="pose-section">
        <h4>Orientation (deg)</h4>
        <div className="pose-values">
          <div className="pose-value">
            <span className="label">Roll:</span>
            <span className="value">{toDegrees(roll)}°</span>
          </div>
          <div className="pose-value">
            <span className="label">Pitch:</span>
            <span className="value">{toDegrees(pitch)}°</span>
          </div>
          <div className="pose-value">
            <span className="label">Yaw:</span>
            <span className="value">{toDegrees(yaw)}°</span>
          </div>
        </div>
      </div>

      <div className="pose-section">
        <h4>Quaternion (w,x,y,z)</h4>
        <div className="pose-values quaternion">
          <span>{formatNumber(w)}</span>
          <span>{formatNumber(x)}</span>
          <span>{formatNumber(y)}</span>
          <span>{formatNumber(z)}</span>
        </div>
      </div>
    </div>
  );
}
