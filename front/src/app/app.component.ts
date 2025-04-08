import { Component, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { Msg, WsService } from './ws.service';
import { FormsModule } from '@angular/forms';
import { LeafletModule } from '@bluehalo/ngx-leaflet';
import { Icon, icon, latLng, Layer, marker, tileLayer } from 'leaflet';
import Chart from 'chart.js/auto';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, FormsModule, LeafletModule],
  templateUrl: './app.component.html',
  styleUrl: './app.component.css'
})
export class AppComponent implements OnInit {
  title = 'front';

  url: string = '';

  layers: Layer[] = [];

  cmds: Msg[] = []
  chart: any = [];

  options = {
    layers: [
      tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18, attribution: '...' })
    ],
    zoom: 5,
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
      console.log(msg);
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
      this.chart.data.labels.push(cmd.kmh);
      this.chart.data.datasets[0].data.push(cmd.kmh);
    });
  }



}
