#!/bin/bash

# Vérification que le script est exécuté à partir de la racine du projet
if [ ! -d ".git" ]; then
    echo "Veuillez exécuter ce script depuis la racine du projet contenant le dépôt Git."
    exit 1
fi

# Affichage d'un message de démarrage
echo "Remplacement des chemins absolus '/home/kylor' par le chemin dynamique \$HOME..."

# Parcourir tous les fichiers dans le projet et remplacer les chemins
grep -rl "/home/kylor" ./ | while read -r file; do
    sed -i "s|/home/kylor|$HOME|g" "$file"
    echo "Chemins corrigés dans le fichier : $file"
done

# Affichage d'un message de fin
echo "Tous les chemins ont été remplacés avec succès !"

# Conseils supplémentaires
echo "Vous pouvez maintenant essayer de relancer votre projet."
