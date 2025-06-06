import { ComponentFixture, TestBed } from '@angular/core/testing';
import { DashboardComponent } from './dashboard.component';
import { RobotService } from '../../services/robot.service';
import { NotificationService } from '../../services/notification.service';
import { WebSocketService } from '../../services/websocket.service';
import { MatDialog, MatDialogRef } from '@angular/material/dialog';
import { NoopAnimationsModule } from '@angular/platform-browser/animations';
import { of } from 'rxjs';
import { ConfirmationDialogComponent } from '../confirmation-dialog/confirmation-dialog.component';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';

describe('DashboardComponent', () => {
  let component: DashboardComponent;
  let fixture: ComponentFixture<DashboardComponent>;
  let robotServiceMock: jasmine.SpyObj<RobotService>;
  let notificationServiceMock: jasmine.SpyObj<NotificationService>;
  let dialogMock: jasmine.SpyObj<MatDialog>;
  let websocketServiceMock: jasmine.SpyObj<WebSocketService>;
  let dialogRefMock: jasmine.SpyObj<MatDialogRef<ConfirmationDialogComponent>>;

  beforeEach(async () => {
    // Créer les mocks
    robotServiceMock = jasmine.createSpyObj('RobotService', [
      'startMission',
      'stopMission',
      'identify',
      'returnToBase',
      'toggleP2P'
    ]);
    
    notificationServiceMock = jasmine.createSpyObj('NotificationService', [
      'missionStarted',
      'missionEnded',
      'identifySignal',
      'returnToBase',
      'p2pStateChanged',
      'warning'
    ]);
    
    dialogRefMock = jasmine.createSpyObj('MatDialogRef', ['afterClosed']);
    dialogRefMock.afterClosed.and.returnValue(of(true)); // Simuler une confirmation
    
    dialogMock = jasmine.createSpyObj('MatDialog', ['open']);
    dialogMock.open.and.returnValue(dialogRefMock);
    
    websocketServiceMock = jasmine.createSpyObj('WebSocketService', [
      'onBatteryData',
      'onRobotState',
      'onRobotPosition',
      'isControllerClient',
      'onClientCountUpdate',
      'emit',
      'onMapData',
      'onMissionLogs'
    ]);
    
    // Configurer les retours des méthodes mockées
    robotServiceMock.startMission.and.returnValue(of({ missionId: 'mission-123' }));
    robotServiceMock.stopMission.and.returnValue(of({ stoppedMissionId: 'mission-123' }));
    robotServiceMock.identify.and.returnValue(of(undefined)); // Correct pour Observable<void>
    robotServiceMock.returnToBase.and.returnValue(of(undefined)); // Correct pour Observable<void>
    robotServiceMock.toggleP2P.and.returnValue(of({ message: 'P2P state toggled successfully' }));
    
    websocketServiceMock.onBatteryData.and.returnValue(of({ robotId: 'limo1', battery_level: 80 }));
    websocketServiceMock.onRobotState.and.returnValue(of({ robotId: 'limo1', state: 'en attente' }));
    websocketServiceMock.onRobotPosition.and.returnValue(of({
      robotId: 'limo1',
      position: { x: 1.2, y: 3.4 }
    }));
    websocketServiceMock.isControllerClient.and.returnValue(true);
    websocketServiceMock.onClientCountUpdate.and.returnValue(of({ count: 3, isController: true })); // Mock avec le format correct
    websocketServiceMock.onMapData.and.returnValue(of({
      resolution: 0.05,
      width: 100,
      height: 100,
      origin: {
        x: 0,
        y: 0,
        z: 0,
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 1
        }
      },
      data: new Array(100 * 100).fill(0)
    }));
    websocketServiceMock.emit.and.returnValue(); // Mock pour la méthode emit
    websocketServiceMock.onMissionLogs.and.returnValue(of([
      {
        type: 'COMMAND',
        robotId: 'limo1',
        data: {
          command: 'START_MISSION',
          timestamp: new Date().toISOString()
        }
      }
    ]));
    
    await TestBed.configureTestingModule({
      imports: [NoopAnimationsModule],
      schemas: [CUSTOM_ELEMENTS_SCHEMA], // Pour ignorer les éléments personnalisés dans le template
      providers: [
        { provide: RobotService, useValue: robotServiceMock },
        { provide: NotificationService, useValue: notificationServiceMock },
        { provide: MatDialog, useValue: dialogMock },
        { provide: WebSocketService, useValue: websocketServiceMock }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(DashboardComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('devrait être créé', () => {
    expect(component).toBeTruthy();
  });

  it('devrait initialiser les états des robots', () => {
    expect(component.robotStates['limo1']).toBeDefined();
    expect(component.robotStates['limo2']).toBeDefined();
    expect(component.robotStates['limo1'].isMissionActive).toBeFalse();
    expect(component.robotStates['limo1'].isIdentified).toBeFalse();
  });

  it('devrait retourner correctement si des robots sont en mission', () => {
    expect(component.anyRobotInMission()).toBeFalse();
    component.robotStates['limo1'].isMissionActive = true;
    expect(component.anyRobotInMission()).toBeTrue();
  });

  it('devrait retourner correctement si tous les robots sont identifiés', () => {
    expect(component.allRobotsIdentified()).toBeFalse();
    component.robotStates['limo1'].isIdentified = true;
    component.robotStates['limo2'].isIdentified = true;
    expect(component.allRobotsIdentified()).toBeTrue();
  });

  it('devrait recevoir correctement les données de batterie', () => {
    expect(component.robotStates['limo1'].battery_level).toBe(80);
    websocketServiceMock.onBatteryData().subscribe(data => {
      expect(data.battery_level).toBe(80);
    });
  });
});
