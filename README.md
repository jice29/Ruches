# Ruches Suite

Repository regroupant:
- `Ruches` (emetteur LoRa/Heltec)
- `ruches-recepteur` (recepteur LoRa + MQTT)
- `ruche-dashboard` (dashboard web Node.js)

## Prerequis
- PlatformIO (VS Code)
- Node.js 18+

## Secrets
- Ne pas committer `secrets.ini`.
- Utiliser `ruches-recepteur/secrets.example.ini` comme modele.

## Lancer le dashboard
```bash
cd ruche-dashboard
npm install
node server.js
```

## Publier sur GitHub
```bash
cd C:\Users\JiCe\Documents\GitHub\ruches-suite
git init
git add .
git commit -m "Initial import: ruches suite"
git branch -M main
git remote add origin https://github.com/<ton-user>/<ton-repo>.git
git push -u origin main
```
