# Architecture Visuelle - Améliorations Esthétiques

## Palette de Couleurs
```css
:root {
  --primary: #2A5C82;    /* Bleu profond */
  --secondary: #5DA271;  /* Vert naturel */
  --accent: #FFB74D;     /* Orange doux */
  --background: #F8F9FA; /* Fond clair */
  --surface: #FFFFFF;    /* Surface blanche */
  --error: #D32F2F;      /* Rouge d'erreur */
  --success: #388E3C;    /* Vert de succès */
  --text-primary: #212121; /* Texte principal */
  --text-secondary: #757575; /* Texte secondaire */
}
```

## Typographie
- **Polices** : 
  - `Roboto` pour le contenu principal
  - `Roboto Condensed` pour les titres
- **Hiérarchie** :
  - H1: 2.5rem (40px)
  - H2: 2rem (32px)
  - Corps de texte: 1rem (16px)
  - Labels: 0.875rem (14px)

## Espacement
- **Grille de base** : 8px
- **Marges** :
  - Petit: 8px
  - Moyen: 16px
  - Grand: 24px
- **Padding des composants** :
  - Cartes: 16px
  - Boutons: 8px 16px
  - Inputs: 12px

## Effets Visuels
```css
.shadow-1 {
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.shadow-2 {
  box-shadow: 0 4px 8px rgba(0,0,0,0.12);
}

.transition-all {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}
```

## Composants Clés

### Navbar
- Hauteur: 64px
- Ombre portée légère
- Menu déroulant avec animations
- Indicateur d'état des robots

### Cartes de Dashboard
- Bordures arrondies: 12px
- Gradient linéaire sur en-tête
- Hover effect avec élévation
- Grille responsive

### Tableaux d'Historique
- Lignes zébrées
- Survol highlight
- Colonnes fixes pour statuts
- Pagination stylisée

## Responsive Design
- Breakpoints :
  - Mobile: < 600px
  - Tablet: 600-1024px
  - Desktop: > 1024px
- Adaptations :
  - Masquage colonnes secondaires
  - Réduction espacements
  - Menu hamburger

## Guide d'Implémentation
1. Créer un theme Angular Material personnalisé
2. Utiliser les variables CSS globales
3. Implémenter les mixins SASS pour les composants
4. Ajouter les animations de transition
5. Tester les contrastes WCAG