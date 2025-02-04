import { Injectable } from '@angular/core';
import { MatSnackBar } from '@angular/material/snack-bar';

@Injectable({
    providedIn: 'root'
})
export class NotificationService {
    constructor(private snackBar: MatSnackBar) {}

    success(message: string, duration: number = 3000): void {
        this.snackBar.open(message, 'Fermer', {
            duration: duration,
            panelClass: ['success-snackbar'],
            horizontalPosition: 'end',
            verticalPosition: 'bottom'
        });
    }

    error(message: string, duration: number = 5000): void {
        this.snackBar.open(message, 'Fermer', {
            duration: duration,
            panelClass: ['error-snackbar'],
            horizontalPosition: 'end',
            verticalPosition: 'bottom'
        });
    }

    info(message: string, duration: number = 3000): void {
        this.snackBar.open(message, 'Fermer', {
            duration: duration,
            panelClass: ['info-snackbar'],
            horizontalPosition: 'end',
            verticalPosition: 'bottom'
        });
    }

    warning(message: string, duration: number = 4000): void {
        this.snackBar.open(message, 'Fermer', {
            duration: duration,
            panelClass: ['warning-snackbar'],
            horizontalPosition: 'end',
            verticalPosition: 'bottom'
        });
    }

    // Pour les erreurs de WebSocket ou autres erreurs de connexion
    connectionError(message: string = 'Erreur de connexion au serveur'): void {
        this.error(`${message}. Tentative de reconnexion...`);
    }

    // Pour les changements de mode de roues
    wheelModeChanged(robotName: string, mode: string): void {
        this.success(`Mode de roues changé pour ${robotName}: ${mode}`);
    }

    // Pour les actions de mission
    missionStarted(): void {
        this.success('Mission démarrée avec succès');
    }

    missionEnded(): void {
        this.info('Mission terminée');
    }

    missionError(error: string): void {
        this.error(`Erreur de mission: ${error}`);
    }

    // Pour les actions de robot
    robotIdentified(robotName: string): void {
        this.info(`Robot ${robotName} identifié`);
    }

    robotReturning(robotName: string): void {
        this.info(`${robotName} retourne à la base`);
    }

    batteryLow(robotName: string, level: number): void {
        this.warning(`Batterie faible pour ${robotName}: ${level}%`);
    }

    // Pour les actions P2P
    p2pEnabled(): void {
        this.success('Communication P2P activée');
    }

    p2pDisabled(): void {
        this.info('Communication P2P désactivée');
    }

    p2pError(error: string): void {
        this.error(`Erreur P2P: ${error}`);
    }
}