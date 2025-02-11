import { Component, OnInit, OnDestroy } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Subscription } from 'rxjs';
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatSelectModule } from '@angular/material/select';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatOptionModule } from '@angular/material/core';
import { RobotService } from '../../services/robot.service';
import { NotificationService } from '../../services/notification.service';
import { Robot, WheelMode } from '../../models/robot.model';

@Component({
  selector: 'app-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.css'],
  standalone: true,
  imports: [
    CommonModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatSelectModule,
    MatFormFieldModule,
    MatOptionModule
  ]
})
export class DashboardComponent implements OnInit, OnDestroy {
  // Ajout des méthodes manquantes
  robots: Robot[] = [];
  isMissionActive = false;
  isP2PEnabled = false;
  private robotsSubscription!: Subscription;
  private selectedRobots: string[] = [];

  constructor(
    private robotService: RobotService,
    private notificationService: NotificationService
  ) {}

  ngOnInit(): void {
    this.robotsSubscription = this.robotService.getRobots()
      .subscribe(robots => {
        // Conversion des RobotState en Robot
        this.robots = robots.map(state => ({
          id: state.id,
          name: state.name,
          state: state
        }));
        this.selectedRobots = this.robots.map(r => r.id);
      });
  }

  getBatteryClass(level: number): string {
    if (level > 70) return 'battery-high';
    if (level > 30) return 'battery-medium';
    return 'battery-low';
  }

  identifyRobot(robotId: string): void {
    this.notificationService.robotIdentified(robotId);
  }

  ngOnDestroy(): void {
    if (this.robotsSubscription) {
      this.robotsSubscription.unsubscribe();
    }
  }

  toggleRobotSelection(robotId: string): void {
    const index = this.selectedRobots.indexOf(robotId);
    if (index > -1) {
      this.selectedRobots.splice(index, 1);
    } else {
      this.selectedRobots.push(robotId);
    }
  }

  isSelected(robotId: string): boolean {
    return this.selectedRobots.includes(robotId);
  }

  startMission(): void {
    if (this.selectedRobots.length > 0) {
      this.robotService.startMission(this.selectedRobots);
      this.notificationService.missionStarted();
    }
  }

  stopMission(): void {
    if (this.selectedRobots.length > 0) {
      this.robotService.stopMission(this.selectedRobots);
      this.notificationService.missionStopped();
    }
  }

  returnToBase(): void {
    if (this.selectedRobots.length > 0) {
      this.robotService.returnToBase(this.selectedRobots);
      this.notificationService.returningToBase();
    }
  }

  setWheelMode(robotId: string, mode: WheelMode): void {
    this.robotService.setWheelMode(robotId, mode);
    this.notificationService.wheelModeChanged(`Robot ${robotId}`, mode);
  }

  toggleP2P(): void {
    if (this.selectedRobots.length > 0) {
      const someP2PEnabled = this.robots.some(r => r.state.p2pEnabled);
      if (someP2PEnabled) {
        this.robotService.disableP2P(this.selectedRobots);
        this.notificationService.p2pDisabled();
      } else {
        this.robotService.enableP2P(this.selectedRobots);
        this.notificationService.p2pEnabled();
      }
    }
  }

  getRobotStatusClass(robot: Robot): string {
    switch (robot.state.status) {
      case 'on_mission':
        return 'status-active';
      case 'returning':
        return 'status-returning';
      case 'error':
        return 'status-error';
      default:
        return 'status-idle';
    }
  }
}
