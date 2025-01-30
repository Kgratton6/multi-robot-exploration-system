import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { NavbarComponent } from './components/navbar/navbar.component';
import { WebSocketService } from './services/websocket.service';

@Component({
    selector: 'app-root',
    standalone: true,
    imports: [
        RouterOutlet,
        NavbarComponent
    ],
    template: `
        <app-navbar></app-navbar>
        <main>
            <router-outlet></router-outlet>
        </main>
    `,
    styles: [`
        :host {
            display: block;
            height: 100vh;
        }

        main {
            height: calc(100vh - 64px);
            overflow-y: auto;
        }
    `]
})
export class AppComponent {
    constructor(private wsService: WebSocketService) {}
}
