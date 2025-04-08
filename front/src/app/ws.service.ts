import { Injectable } from '@angular/core';
import { map, Observable } from 'rxjs';
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';

export interface Msg {
  lat: number;
  lon: number;
  kmh: number;
  x: number;
  y: number;
  rssi: number;
}

@Injectable({
  providedIn: 'root',
})
export class WsService {
  subject: WebSocketSubject<Msg> | null = null;
  latLng: Observable<Pick<Msg, 'lat' | 'lon'>> | null = null;
  rssi: Observable<Pick<Msg, 'rssi'>> | null = null;

  constructor() { }

  setServer(server: string) {
    this.subject = webSocket(server);
    this.subject.subscribe({
      next: msg => console.log('message received: ' + msg), // Called whenever there is a message from the server.
      error: err => console.log(err), // Called if at any point WebSocket API signals some kind of error.
      complete: () => console.log('complete') // Called when connection is closed (for whatever reason).
    });

    this.latLng = this.subject.pipe(map((msg) => ({ lat: msg.lat, lon: msg.lon })));
    this.rssi = this.subject.pipe(map((msg) => ({ rssi: msg.rssi })));

  }
}