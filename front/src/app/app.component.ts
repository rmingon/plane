import { Component, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { Msg, WsService } from './ws.service';
import { FormsModule } from '@angular/forms';
import { LeafletModule } from '@bluehalo/ngx-leaflet';
import { Icon, icon, latLng, Layer, marker, tileLayer } from 'leaflet';
import Chart from 'chart.js/auto';
import { ThreeSceneComponent } from "./three-scene/three-scene.component";

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, FormsModule, LeafletModule, ThreeSceneComponent],
  templateUrl: './app.component.html',
  styleUrl: './app.component.css'
})
export class AppComponent implements OnInit {
  title = 'front';

  url: string = '';

  layers: Layer[] = [];

  cmds: Msg[] = []
  chart: any = [];

  roll: number = 0;
  pitch: number = 0;
  yaw: number = 0;
  deg: number = 0;

  options = {
    layers: [
      tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18, attribution: '...' })
    ],
    zoom: 10,
    center: latLng(45.750000, 4.850000)
  };

  constructor(private wsService: WsService) { }
  ngOnInit() {
    this.chart = new Chart('canvas', {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'rssi',
            data: [],
            borderWidth: 1,
          },
          {
            label: 'speed',
            data: [],
            borderWidth: 1,
          },
        ],
      },
      options: {
        scales: {
          y: {
            beginAtZero: true,
          },
        },
      },
    });

  }

  addServerUrl() {
    this.wsService.setServer(this.url);

    this.wsService.latLng?.subscribe((msg) => {
      this.layers.push(marker([msg.lat, msg.lon], {
        icon: icon({
          ...Icon.Default.prototype.options,
          iconUrl: 'assets/marker-icon.png',
          iconRetinaUrl: 'assets/marker-icon-2x.png',
          shadowUrl: 'assets/marker-shadow.png'
        }),
      }));
      this.options.center = latLng(msg.lat, msg.lon);
      this.options.zoom = 10;
    });

    this.wsService.subject?.subscribe((cmd) => {
      if (this.chart.data.labels.length > 50) {
        this.chart.data.labels.shift();
        this.chart.data.datasets[0].data.shift();
        this.chart.data.datasets[1].data.shift();
      }
      this.chart.data.labels.push(new Date().toLocaleTimeString());
      this.chart.data.datasets[0].data.push(cmd.rssi);
      this.chart.data.datasets[1].data.push(cmd.kmh);
      this.chart.update();
      this.roll = cmd.x;
      this.deg = cmd.magY;
    });
  }



}
