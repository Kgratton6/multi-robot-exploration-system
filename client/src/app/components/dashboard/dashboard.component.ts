import { Component, OnInit, OnDestroy } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatIconModule } from '@angular/material/icon';
import { MatSelectModule } from '@angular/material/select';
import { MatTooltipModule } from '@angular/material/tooltip';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { FormsModule } from '@angular/forms';
import { Subject, takeUntil } from 'rxjs';
import { Robot, WheelMode } from '../../models/robot.model';
import { RobotService } from '../../services/robot.service';
import { MissionService } from '../../services/mission.service';
import { NotificationService } from '../../services/notification.service';
import { ConfirmationDialogComponent } from '../confirmation-dialog/confirmation-dialog.component';

@Component({
    selector: 'app-dashboard',
    standalone: true,
    imports: [
        CommonModule,
        MatButtonModule,
        MatCardModule,
        MatIconModule,
        MatSelectModule,
        MatTooltipModule,
        MatDialogModule,
        FormsModule,
        // ConfirmationDialogComponent
    ],
    templateUrl: './dashboard.component.html',
    styleUrl: './dashboard.component.css'
})
export class DashboardComponent implements OnInit, OnDestroy {
    robots: Robot[] = [];
    isP2PEnabled = false;
    isMissionActive = false;
    private destroy$ = new Subject<void>();

    constructor(
        private robotService: RobotService,
        private missionService: MissionService,
        private notificationService: NotificationService,
        private dialog: MatDialog,
    ) {}

    ngOnInit() {
        // Subscribe to robot updates
        this.robotService.getRobots()
            .pipe(takeUntil(this.destroy$))
            .subscribe(robots => {
                this.robots = robots;
            });

        // Subscribe to mission status
        this.missionService.getCurrentMission()
            .pipe(takeUntil(this.destroy$))
            .subscribe(mission => {
                this.isMissionActive = !!mission;
            });
    }

    ngOnDestroy() {
        this.destroy$.next();
        this.destroy$.complete();
    }

    identifyRobot(robotId: string): void {
        this.robotService.identifyRobot(robotId);
        this.notificationService.robotIdentified(`Robot ${robotId}`);
    }

    startMission(): void {
        this.missionService.startNewMission();
        this.robotService.startMission();
        this.notificationService.missionStarted();
    }

    stopMission(): void {
        const dialogRef = this.dialog.open(ConfirmationDialogComponent, {
            width: '400px',
            data: { message: 'Êtes-vous sûr de vouloir arrêter la mission en cours ?' }
        });

        dialogRef.afterClosed().subscribe(result => {
            if (result) {
                this.robotService.stopMission();
                this.missionService.endCurrentMission();
                this.notificationService.missionEnded();
            }
        });
    }

    returnToBase(): void {
        this.robotService.returnToBase();
        this.notificationService.info('Les robots retournent à la base');
    }

    setWheelMode(robotId: string, mode: WheelMode): void {
        this.robotService.setWheelMode(robotId, mode);
        this.notificationService.wheelModeChanged(`Robot ${robotId}`, mode.type);
    }

    toggleP2P(): void {
        this.isP2PEnabled = !this.isP2PEnabled;
        if (this.isP2PEnabled) {
            this.robotService.enableP2P();
            this.notificationService.p2pEnabled();
        } else {
            this.robotService.disableP2P();
            this.notificationService.p2pDisabled();
        }
    }

    getRobotStatusClass(robot: Robot): string {
        switch (robot.state.status) {
            case 'on_mission':
                return 'status-active';
            case 'returning':
                return 'status-returning';
            case 'charging':
                return 'status-charging';
            default:
                return 'status-waiting';
        }
    }

    getBatteryClass(level: number): string {
        if (level <= 20) return 'battery-critical';
        if (level <= 30) return 'battery-low';
        return 'battery-normal';
    }
}
