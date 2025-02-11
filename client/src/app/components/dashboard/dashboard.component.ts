import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { RobotService } from '../../services/robot.service';
import { NotificationService } from '../../services/notification.service';
import { ConfirmationDialogComponent } from '../confirmation-dialog/confirmation-dialog.component';

@Component({
    selector: 'app-dashboard',
    standalone: true,
    imports: [
        CommonModule,
        MatButtonModule,
        MatCardModule,
        MatDialogModule
    ],
    templateUrl: './dashboard.component.html',
    styleUrl: './dashboard.component.css'
})
export class DashboardComponent {
    isMissionActive = false;

    constructor(
        private robotService: RobotService,
        private notificationService: NotificationService,
        private dialog: MatDialog,
    ) {}

    startMission(): void {
        this.robotService.startMission().subscribe(() => {
            this.isMissionActive = true;
            this.notificationService.missionStarted();
        });
    }

    stopMission(): void {
        const dialogRef = this.dialog.open(ConfirmationDialogComponent, {
            width: '400px',
            data: { message: 'Êtes-vous sûr de vouloir arrêter la mission en cours ?' }
        });

        dialogRef.afterClosed().subscribe(result => {
            if (result) {
                this.robotService.stopMission().subscribe(() => {
                    this.isMissionActive = false;
                    this.notificationService.missionEnded();
                });
            }
        });
    }
}
