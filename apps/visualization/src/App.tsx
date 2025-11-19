import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Environment } from '@react-three/drei';
import { Suspense, useEffect, useState } from 'react';
import { loadUrdfxModule } from './lib/urdfx-loader';
import { RobotRenderer } from './components/RobotRenderer';
import { Leva } from 'leva';

function App() {
  const [urdfx, setUrdfx] = useState<any>(null);
  const [urdfContent, setUrdfContent] = useState<string | null>(null);

  useEffect(() => {
    loadUrdfxModule().then((mod) => {
      setUrdfx(mod);
    });
  }, []);

  const handleFileUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      const reader = new FileReader();
      reader.onload = (ev) => {
        setUrdfContent(ev.target?.result as string);
      };
      reader.readAsText(file);
    }
  };

  if (!urdfx) return <div>Loading WASM...</div>;

  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      <div style={{ position: 'absolute', top: 10, left: 10, zIndex: 1000, background: 'white', padding: 10, borderRadius: 4 }}>
        <input type="file" accept=".urdf" onChange={handleFileUpload} />
      </div>
      <Leva />
      <Canvas camera={{ position: [2, 2, 2], fov: 50 }}>
        <Suspense fallback={null}>
          <Environment preset="city" />
          <Grid infiniteGrid />
          <OrbitControls makeDefault />
          <RobotRenderer urdfx={urdfx} urdfUrl="/ur5e.urdf" urdfContent={urdfContent} />
        </Suspense>
      </Canvas>
    </div>
  );
}

export default App;
