import { useEffect, useRef } from 'react';
import * as THREE from 'three';

export const Rocket3D = ({ q }) => {
    const containerRef = useRef();
    const rocketRef    = useRef();
    
    useEffect(() => {
        const scene    = new THREE.Scene();
        const camera   = new THREE.PerspectiveCamera(40, 1, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
        renderer.setSize(140, 140);
        
        // 確保不會因為 React.StrictMode 執行兩次而產生兩個 canvas
        if (containerRef.current) {
            containerRef.current.innerHTML = '';
            containerRef.current.appendChild(renderer.domElement);
        }
        
        const rfuWorldGroup = new THREE.Group();
        // 依據您的需求：世界座標為 RFU (X右, Y入螢幕, Z上)
        // Three.js 預設為：X右, Y上, Z出螢幕
        // RFU X (右) -> Three +X (右)
        // RFU Y (前 / 入螢幕) -> Three -Z (入螢幕)
        // RFU Z (上) -> Three +Y (上)
        const matrix = new THREE.Matrix4();
        matrix.makeBasis(
            new THREE.Vector3(1, 0, 0),    // 映射 RFU X
            new THREE.Vector3(0, 0, -1),   // 映射 RFU Y
            new THREE.Vector3(0, 1, 0)     // 映射 RFU Z
        );
        rfuWorldGroup.setRotationFromMatrix(matrix);
        scene.add(rfuWorldGroup);

        const rfuRocket = new THREE.Group();
        
        // 1. Build the rocket in the new Body Frame (X=Right, Y=Bottom, Z=Nose)
        // Cylinder along Z axis
        const bodyGeo = new THREE.CylinderGeometry(0.25, 0.25, 2.2, 32);
        bodyGeo.rotateX(Math.PI / 2); // Rotate to align with Z axis
        rfuRocket.add(new THREE.Mesh(bodyGeo, new THREE.MeshPhongMaterial({ color:0xdddddd })));
        
        // Nose cone at +Z
        const noseGeo = new THREE.ConeGeometry(0.25, 0.6, 32);
        noseGeo.rotateX(Math.PI / 2);
        noseGeo.translate(0, 0, 1.4);
        rfuRocket.add(new THREE.Mesh(noseGeo, new THREE.MeshPhongMaterial({ color:0xff0000 })));

        // Top fin (Red) at -Y
        const finGeoY = new THREE.BoxGeometry(0.05, 0.4, 0.6); 
        const topFin = new THREE.Mesh(finGeoY, new THREE.MeshPhongMaterial({ color:0xff0000 }));
        topFin.position.set(0, -0.35, -0.8);
        rfuRocket.add(topFin);
        
        // Belly fin at +Y
        const bottomFin = new THREE.Mesh(finGeoY, new THREE.MeshPhongMaterial({ color:0x555555 }));
        bottomFin.position.set(0, 0.35, -0.8);
        rfuRocket.add(bottomFin);
        
        // Left fin at -X
        const finGeoX = new THREE.BoxGeometry(0.4, 0.05, 0.6);
        const leftFin = new THREE.Mesh(finGeoX, new THREE.MeshPhongMaterial({ color:0x555555 }));
        leftFin.position.set(-0.35, 0, -0.8);
        rfuRocket.add(leftFin);
        
        // Right fin at +X
        const rightFin = new THREE.Mesh(finGeoX, new THREE.MeshPhongMaterial({ color:0x555555 }));
        rightFin.position.set(0.35, 0, -0.8);
        rfuRocket.add(rightFin);

        rfuWorldGroup.add(rfuRocket); 
        rocketRef.current = rfuRocket;
        
        const dl = new THREE.DirectionalLight(0xffffff, 1.2); 
        dl.position.set(5, 5, 5); 
        scene.add(dl);
        
        scene.add(new THREE.AmbientLight(0x222222)); 
        camera.position.set(2, 2, 4.5);
        camera.lookAt(0, 0, 0);
        
        let animationFrameId;
        const animate = () => { 
            animationFrameId = requestAnimationFrame(animate); 
            renderer.render(scene, camera); 
        };
        animate();
        
        return () => { 
            cancelAnimationFrame(animationFrameId);
            try { containerRef.current.removeChild(renderer.domElement); } catch(e){} 
        };
    }, []);
    
    useEffect(() => {
        if (rocketRef.current && q) {
            // Three.js 的 Quaternion 參數順序固定為 (x, y, z, w)
            // 如果 monitor.py 傳來的是 [w, x, y, z]，必須對應放入才能確保 3D 旋轉數學正確
            rocketRef.current.setRotationFromQuaternion(new THREE.Quaternion(q[1], q[2], q[3], q[0]));
        }
    }, [q]);
    
    return <div ref={containerRef}></div>;
};
