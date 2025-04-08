import { Injectable } from '@angular/core';
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';

@Injectable({
  providedIn: 'root',
})
export class WsService {
    subject: WebSocketSubject<any> | null = null;

  constructor() { }

  setServer(server: string) {
    this.subject = webSocket(server);
    this.subject.subscribe({
        next: msg => console.log('message received: ' + msg), // Called whenever there is a message from the server.
        error: err => console.log(err), // Called if at any point WebSocket API signals some kind of error.
        complete: () => console.log('complete') // Called when connection is closed (for whatever reason).
       });
  }
}