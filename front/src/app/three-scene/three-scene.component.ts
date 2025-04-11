import {
  Component,
  ElementRef,
  AfterViewInit,
  ViewChild,
  OnDestroy,
} from '@angular/core';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';

@Component({
  selector: 'app-three-scene',
  templateUrl: './three-scene.component.html',
  styleUrls: ['./three-scene.component.css'],
})
export class ThreeSceneComponent implements AfterViewInit, OnDestroy {
  @ViewChild('canvasContainer', { static: false }) containerRef!: ElementRef;

  private scene!: THREE.Scene;
  private camera!: THREE.PerspectiveCamera;
  private renderer!: THREE.WebGLRenderer;
  private animationId!: number;

  ngAfterViewInit(): void {
    this.initThreeJS();
    this.loadObjModel();
    this.animate();
  }

  private initThreeJS(): void {
    const width = this.containerRef.nativeElement.clientWidth;
    const height = this.containerRef.nativeElement.clientHeight;

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    this.camera.position.z = 5;

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.containerRef.nativeElement.appendChild(this.renderer.domElement);

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(0, 1, 1).normalize();
    this.scene.add(light);
  }

  private loadObjModel(): void {
    const mtlLoader = new MTLLoader();
    mtlLoader.setPath('models/');
    mtlLoader.load('plane.mtl', (materials: any) => {
      materials.preload();

      const objLoader = new OBJLoader();
      objLoader.setMaterials(materials);
      objLoader.setPath('models/');
      objLoader.load(
        'plane.obj',
        (object: any) => {
          object.scale.set(0.1, 0.1, 0.1); // Adjust scale if needed
          this.scene.add(object);
          object.position.set(0, 0, 0); // Adjust position if needed
        },
        (xhr: any) => {
          console.log((xhr.loaded / xhr.total) * 100 + '% loaded');
        },
        (error: any) => {
          console.error('Error loading OBJ model:', error);
        }
      );
    });
  }

  private animate = (): void => {
    this.animationId = requestAnimationFrame(this.animate);
    this.renderer.render(this.scene, this.camera);
  };

  ngOnDestroy(): void {
    cancelAnimationFrame(this.animationId);
    if (this.renderer) this.renderer.dispose();
  }
}