import { Injectable } from '@angular/core';
import { MatSnackBar } from '@angular/material/snack-bar';

@Injectable({
  providedIn: 'root'
})
export class NotificationService {
  constructor(private snackBar: MatSnackBar) {}

  missionStarted(): void {
    this.showNotification('Mission démarrée');
  }

  missionStopped(): void {
    this.showNotification('Mission arrêtée');
  }

  returningToBase(): void {
    this.showNotification('Retour à la base');
  }

  wheelModeChanged(robotId: string, mode: string): void {
    this.showNotification(`Mode de roues changé pour ${robotId}: ${mode}`);
  }

  p2pEnabled(): void {
    this.showNotification('Communication P2P activée');
  }

  p2pDisabled(): void {
    this.showNotification('Communication P2P désactivée');
  }

  robotIdentified(robotId: string): void {
    this.showNotification(`Robot ${robotId} identifié`);
  }

  error(message: string): void {
    this.showNotification(`Erreur: ${message}`, 'error');
  }

  private showNotification(message: string, type: 'success' | 'error' = 'success'): void {
    this.snackBar.open(message, 'Fermer', {
      duration: 3000,
      horizontalPosition: 'right',
      verticalPosition: 'top',
      panelClass: type === 'error' ? 'error-snackbar' : 'success-snackbar'
    });
  }
}