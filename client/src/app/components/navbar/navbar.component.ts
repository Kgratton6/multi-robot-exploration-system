import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RouterModule } from '@angular/router';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';

@Component({
    selector: 'app-navbar',
    standalone: true,
    imports: [
        CommonModule,
        RouterModule,
        MatToolbarModule,
        MatButtonModule,
        MatIconModule
    ],
    templateUrl: './navbar.component.html',
    styleUrl: './navbar.component.css'
})
export class NavbarComponent {
    navItems = [
        {
            path: '/dashboard',
            label: 'Tableau de bord',
            icon: 'dashboard'
        },
        {
            path: '/missions',
            label: 'Historique',
            icon: 'history'
        },
        {
            path: '/configuration',
            label: 'Configuration',
            icon: 'settings'
        }
    ];
}