# Frontend Application - Système Multi-Robot

## Architecture

L'application est construite avec Angular 19 et utilise l'architecture suivante :

### Core Components

- `DashboardComponent` : Interface principale pour le contrôle des robots et la visualisation de la carte
- `MissionHistoryComponent` : Historique des missions avec détails et statistiques
- `ConfigurationComponent` : Configuration des modes de roues et autres paramètres
- `NavbarComponent` : Navigation principale de l'application

### Services

- `WebSocketService` : Gestion de la communication en temps réel avec le back-end
- `RobotService` : Gestion des robots et de leurs états
- `MissionService` : Gestion des missions et de leur historique
- `NotificationService` : Système de notifications unifié

### Models

- `Robot` : Interface décrivant la structure d'un robot et son état
- `Mission` : Interface décrivant une mission et ses métadonnées
- `WheelMode` : Type pour les différents modes de roues (Ackerman/Différentiel)

### Features

- Communication en temps réel via WebSocket
- Affichage de l'état des robots en direct
- Historique des missions avec filtrage
- Configuration des modes de roues
- Support P2P entre robots
- Interface responsive (mobile-friendly)
- Gestion des erreurs centralisée

## Prérequis

- Node.js (v18+)
- npm ou yarn
- Angular CLI 19+

## Installation

```bash
# Installation des dépendances
npm install

# Démarrage en mode développement
npm start

# Build pour la production
npm run build
```

## Structure des Dossiers

```
src/
├── app/
│   ├── components/        # Composants de l'application
│   ├── services/         # Services partagés
│   ├── models/           # Interfaces et types
│   ├── interceptors/     # Intercepteurs HTTP
│   └── ...
├── environments/         # Configuration par environnement
├── assets/              # Ressources statiques
└── styles.css           # Styles globaux
```

## Requis Implémentés

### Requis Obligatoires
- [x] R.F.1 - Identification des robots
- [x] R.F.2 - Commandes de mission (démarrer/arrêter)
- [x] R.F.3 - Affichage de l'état des robots
- [x] R.F.4 - Navigation autonome
- [x] R.F.5 - Évitement d'obstacles
- [x] R.F.6 - Retour à la base
- [x] R.F.7 - Gestion de la batterie
- [x] R.F.8 - Cartographie
- [x] R.F.9 - Affichage des positions
- [x] R.F.10 - Interface Web responsive

### Requis Optionnels Choisis
- [x] R.F.15 - Modes de roues différents
- [x] R.F.17 - Base de données des missions
- [x] R.F.18 - Sauvegarde des cartes
- [x] R.F.19 - Communication P2P

## Conventions de Code

Le projet suit les conventions de code Angular officielles et utilise :
- ESLint pour le linting
- Prettier pour le formatage
- Angular Material pour les composants UI
- RxJS pour la gestion des flux de données

## Tests

Les tests sont écrits avec Jasmine et peuvent être exécutés avec :

```bash
# Tests unitaires
npm test

# Tests end-to-end
npm run e2e
```

## Conteneurisation

L'application est conteneurisée avec Docker. Pour construire et exécuter :

```bash
# Build de l'image
docker build -t robot-frontend .

# Exécution du conteneur
docker run -p 80:80 robot-frontend
```

## Communication avec le Back-end

L'application communique avec le back-end via :
- WebSocket pour les données en temps réel (état des robots, positions, etc.)
- API REST pour les opérations CRUD (missions, configurations, etc.)

Les URLs sont configurables dans les fichiers d'environnement.
