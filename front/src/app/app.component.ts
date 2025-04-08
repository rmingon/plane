import { Component, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { WsService } from './ws.service';
import { FormsModule } from '@angular/forms';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, FormsModule],
  templateUrl: './app.component.html',
  styleUrl: './app.component.css'
})
export class AppComponent implements OnInit {
  title = 'front';

  url: string = '';

  constructor(private wsService: WsService) {}
    ngOnInit() {
  }

  addServerUrl() {
    console.log(this.url);
    this.wsService.setServer(this.url);
  }
  
}
