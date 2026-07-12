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
        
        const group = new THREE.Group();
        group.add(new THREE.Mesh(new THREE.CylinderGeometry(0.25,0.25,2.2,32), new THREE.MeshPhongMaterial({ color:0xdddddd })));
        
        const nose = new THREE.Mesh(new THREE.ConeGeometry(0.25,0.6,32), new THREE.MeshPhongMaterial({ color:0xff0000 }));
        nose.position.y = 1.4; 
        group.add(nose);

        // 加入尾翼作為方向標示
        const finGeoZ = new THREE.BoxGeometry(0.05, 0.6, 0.4);
        const finGeoX = new THREE.BoxGeometry(0.4, 0.6, 0.05);
        
        // 紅色尾翼 (放在 +Z 軸) 也就是「正面」標示
        const finFront = new THREE.Mesh(finGeoZ, new THREE.MeshPhongMaterial({ color:0xff0000 }));
        finFront.position.set(0, -0.8, 0.35);
        group.add(finFront);
        
        const finBack = new THREE.Mesh(finGeoZ, new THREE.MeshPhongMaterial({ color:0x555555 }));
        finBack.position.set(0, -0.8, -0.35);
        group.add(finBack);
        
        const finRight = new THREE.Mesh(finGeoX, new THREE.MeshPhongMaterial({ color:0x555555 }));
        finRight.position.set(0.35, -0.8, 0);
        group.add(finRight);
        
        const finLeft = new THREE.Mesh(finGeoX, new THREE.MeshPhongMaterial({ color:0x555555 }));
        finLeft.position.set(-0.35, -0.8, 0);
        group.add(finLeft);

        scene.add(group); 
        rocketRef.current = group;
        
        const dl = new THREE.DirectionalLight(0xffffff,1.2); 
        dl.position.set(5,5,5); 
        scene.add(dl);
        
        scene.add(new THREE.AmbientLight(0x222222)); 
        camera.position.z = 4.5;
        
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
            // 根據實測校準數據：
            // 左傾90度 (Nose Left)  => QX = -0.707
            // 右傾90度 (Nose Right) => QX = +0.707
            // 這代表實體感測器的 X 軸是「朝後 (Into screen)」，Y 軸是「朝左」，Z 軸是「朝上」
            // Three.js 座標系：X 朝右，Y 朝上，Z 朝前 (Out of screen)
            // 映射關係：Three.X = -Phys.Y, Three.Y = Phys.Z, Three.Z = -Phys.X
            // 因此四元數映射為：x' = -q.y, y' = q.z, z' = -q.x, w' = q.w
            rocketRef.current.setRotationFromQuaternion(new THREE.Quaternion(-q[2], q[3], -q[1], q[0]));
        }
    }, [q]);
    
    return <div ref={containerRef}></div>;
};
