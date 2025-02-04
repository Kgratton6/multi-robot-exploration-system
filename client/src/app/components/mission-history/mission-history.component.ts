import { Component, OnInit, OnDestroy } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatTableModule } from '@angular/material/table';
import { MatSortModule } from '@angular/material/sort';
import { MatPaginatorModule } from '@angular/material/paginator';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatDialogModule } from '@angular/material/dialog';
import { MatCardModule } from '@angular/material/card';
import { MatTooltipModule } from '@angular/material/tooltip';
import { Subject, takeUntil } from 'rxjs';
import { Mission } from '../../models/mission.model';
import { MissionService } from '../../services/mission.service';

@Component({
    selector: 'app-mission-history',
    standalone: true,
    imports: [
        CommonModule,
        MatTableModule,
        MatSortModule,
        MatPaginatorModule,
        MatButtonModule,
        MatIconModule,
        MatDialogModule,
        MatCardModule,
        MatTooltipModule
    ],
    templateUrl: './mission-history.component.html',
    styleUrl: './mission-history.component.css'
})
export class MissionHistoryComponent implements OnInit, OnDestroy {
    displayedColumns: string[] = ['startTime', 'duration', 'status', 'robots', 'totalDistance', 'actions'];
    missions: Mission[] = [];
    private destroy$ = new Subject<void>();

    constructor(private missionService: MissionService) {}

    ngOnInit() {
        this.loadMissions();
    }

    ngOnDestroy() {
        this.destroy$.next();
        this.destroy$.complete();
    }

    loadMissions() {
        this.missionService.loadMissionHistory()
            .pipe(takeUntil(this.destroy$))
            .subscribe(missions => {
                this.missions = missions;
            });
    }

    formatDuration(duration: number | undefined): string {
        if (!duration) return '---';
        const minutes = Math.floor(duration / 60);
        const seconds = duration % 60;
        return `${minutes}m ${seconds}s`;
    }

    formatDistance(distance: number): string {
        return `${distance.toFixed(2)}m`;
    }

    viewMissionDetails(mission: Mission): void {
        // TODO: Implémenter l'affichage détaillé de la mission
        console.log('Afficher les détails de la mission:', mission);
    }

    downloadLogs(mission: Mission): void {
        this.missionService.downloadLogs(mission.id)
            .pipe(takeUntil(this.destroy$))
            .subscribe(logs => {
                // Créer un fichier de logs à télécharger
                const blob = new Blob([JSON.stringify(logs, null, 2)], { type: 'application/json' });
                const url = window.URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `mission-${mission.id}-logs.json`;
                document.body.appendChild(a);
                a.click();
                window.URL.revokeObjectURL(url);
                document.body.removeChild(a);
            });
    }

    loadMap(mission: Mission): void {
        if (mission.map) {
            this.missionService.loadMap(mission.id)
                .pipe(takeUntil(this.destroy$))
                .subscribe(mapData => {
                    // TODO: Implémenter l'affichage de la carte
                    console.log('Charger la carte:', mapData);
                });
        }
    }
}