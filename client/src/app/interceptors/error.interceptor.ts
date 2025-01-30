import { inject } from '@angular/core';
import {
    HttpErrorResponse,
    HttpEvent,
    HttpHandlerFn,
    HttpInterceptorFn,
    HttpRequest
} from '@angular/common/http';
import { Observable, throwError } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { NotificationService } from '../services/notification.service';

export const errorInterceptor: HttpInterceptorFn = (
    req: HttpRequest<unknown>,
    next: HttpHandlerFn
): Observable<HttpEvent<unknown>> => {
    const notificationService = inject(NotificationService);

    return next(req).pipe(
        catchError((error: HttpErrorResponse) => {
            let errorMessage = 'Une erreur est survenue';

            if (error.error instanceof ErrorEvent) {
                // Erreur côté client
                errorMessage = `Erreur: ${error.error.message}`;
            } else {
                // Erreur côté serveur
                switch (error.status) {
                    case 400:
                        errorMessage = 'Requête invalide';
                        break;
                    case 401:
                        errorMessage = 'Non autorisé';
                        break;
                    case 403:
                        errorMessage = 'Accès refusé';
                        break;
                    case 404:
                        errorMessage = 'Ressource non trouvée';
                        break;
                    case 408:
                        errorMessage = 'Délai d\'attente dépassé';
                        break;
                    case 500:
                        errorMessage = 'Erreur serveur';
                        break;
                    case 503:
                        errorMessage = 'Service indisponible';
                        break;
                    default:
                        errorMessage = `Erreur ${error.status}: ${error.statusText}`;
                }

                // Ajouter le message d'erreur du serveur s'il existe
                if (error.error?.message) {
                    errorMessage += ` - ${error.error.message}`;
                }
            }

            // Afficher la notification d'erreur
            notificationService.error(errorMessage);

            // Propager l'erreur
            return throwError(() => error);
        })
    );
};