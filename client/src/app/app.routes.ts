import { Routes } from '@angular/router';
import { DashboardComponent } from './components/dashboard/dashboard.component';

export const routes: Routes = [
    {
        path: '',
        redirectTo: 'dashboard',
        pathMatch: 'full'
    },
    {
        path: 'dashboard',
        component: DashboardComponent,
        title: 'Tableau de bord'
    },
    {
        path: 'missions',
        loadChildren: () => import('./components/mission-history/mission-history.routes')
            .then(m => m.MISSION_HISTORY_ROUTES),
        title: 'Historique des missions'
    },
    {
        path: 'configuration',
        loadChildren: () => import('./components/configuration/configuration.routes')
            .then(m => m.CONFIGURATION_ROUTES),
        title: 'Configuration'
    },
    {
        path: '**',
        redirectTo: 'dashboard'
    }
];
