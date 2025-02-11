import { Component, OnDestroy, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatSelectModule } from '@angular/material/select';
import { MatFormFieldModule } from '@angular/material/form-field';
import { FormsModule } from '@angular/forms';
import { Subject, takeUntil } from 'rxjs';
import { Robot, WheelMode } from '../../models/robot.model';
import { RobotService } from '../../services/robot.service';

@Component({
    selector: 'app-configuration',
    standalone: true,
    imports: [
        CommonModule,
        MatCardModule,
        MatButtonModule,
        MatIconModule,
        MatSelectModule,
        MatFormFieldModule,
        FormsModule
    ],
    templateUrl: './configuration.component.html',
    styleUrl: './configuration.component.css'
})
export class ConfigurationComponent implements OnInit, OnDestroy {
    robots: Robot[] = [];
    private destroy$ = new Subject<void>();

    wheelModes: { value: WheelMode['type']; label: string }[] = [
        { value: 'ackerman', label: 'Mode Ackerman' },
        { value: 'differential', label: 'Mode Différentiel' }
    ];

    constructor(private robotService: RobotService) {}

    ngOnInit() {
        this.robotService.getRobots()
            .pipe(takeUntil(this.destroy$))
            .subscribe(robots => {
                this.robots = robots;
            });
    }

    ngOnDestroy() {
        this.destroy$.next();
        this.destroy$.complete();
    }

    onWheelModeChange(robotId: string, mode: WheelMode['type']) {
        this.robotService.setWheelMode(robotId, { type: mode });
    }

    getCurrentWheelMode(robot: Robot): WheelMode['type'] {
        return robot.state.wheelMode.type;
    }

    getWheelModeLabel(mode: WheelMode['type']): string {
        return this.wheelModes.find(m => m.value === mode)?.label || mode;
    }
}