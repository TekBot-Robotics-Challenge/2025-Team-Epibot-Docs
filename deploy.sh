#!/usr/bin/env sh

# abort on errors
set -e

# build
npm run docs:build

# navigate into the build output directory
cd docs/.vitepress/dist

# si tu utilises un custom domain, crée un fichier CNAME ici

git init
git add -A
git commit -m 'deploy'

# push to gh-pages branch
git push -f git@github.com:TekBot-Robotics-Challenge/2025-Team-Epibot-Docs.git main:gh-pages

cd -
