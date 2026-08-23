# CLAUDE.md — Mecanum Platform (ESP32-S3)

Firmware czterokołowej platformy mecanum. Ten plik zawiera wyłącznie rzeczy,
których nie widać z samego kodu: trwałe niezmienniki i decyzje architektoniczne.
Świadomie **nie** ma tu statusu prac, planów ani nazw gałęzi — to rotuje
najszybciej i tworzy fałszywy kontekst.

## Architektura systemu

System składa się z **dwóch** urządzeń:

- **Pad** (Xiao ESP32-S3, repo `Pad_Adafruit_Xiao`) — joysticki, wyświetlacz TFT.
  Pełni rolę pilota **oraz** monitora/debugera.
- **Platforma** (to repo) — cztery silniki DC z enkoderami, PID, kinematyka mecanum.

Osobny moduł „debug monitor" i aplikacja na iPhone zostały **porzucone**
(decyzja z 2026-08-23), żeby szybciej dowieźć działającą całość. Rolę monitora
przejął Pad, bo ma własny wyświetlacz i tak czy owak jest w rękach operatora.

Kod, który jeszcze o nich pamięta — `monitorUpdateTask`, `Message_from_Monitor`,
`macMonitorDebug` i adresat telemetrii w `debugTask` — jest przeznaczony do
przeadresowania na Pada albo do usunięcia. Nie rozwijaj go.

Komunikacja SPI została porzucona razem z monitorem. Jedynym kanałem jest ESP-NOW.

## Niezmienniki, których złamanie nie da błędu kompilacji

To są pułapki ciche — kod się zbuduje, CI przejdzie, a system nie zadziała.

### `src/messages.h` musi być identyczny z kopią w repo Pada

`OnDataRecv` rozpoznaje typ wiadomości **wyłącznie** po `len == sizeof(struct)`.
Zmiana pola w jednym repo bez drugiego nie zepsuje kompilacji — spowoduje ciche
odrzucanie pakietów, a przy przypadkowej zgodności rozmiarów interpretację
danych jako niewłaściwej struktury. Każda zmiana struktury to zmiana w **obu**
repo w tym samym kroku.

### Żadne dwa koła nie mogą mieć identycznego wzoru kinematyki

Obowiązuje (`MecanumDrive::drive`):

    FL = vy + vx + ω        FR = vy − vx − ω
    RL = vy − vx + ω        RR = vy + vx − ω

Jeśli dwa wiersze staną się identyczne, macierz mieszania traci rząd i robot
**fizycznie nie potrafi jechać bokiem** — zamiast tego skręca.

Ten błąd już raz wszedł do repo (commit `d64fbf7`, naprawiony w `76f6cb2`).
Przeszedł, bo opis commita dokładnie zgadzał się z diffem — zmiana była trafnie
opisana i mimo to błędna. Wniosek: weryfikuj wzory względem matematyki mecanum,
nie względem tego, co opis zmiany o sobie twierdzi.

Prawa strona ma `invertDirection = true` w `motor_config.h`, dzięki czemu dla
każdego koła „dodatnie = do przodu" i powyższe wzory obowiązują bez korekt.

### Jazda w przód/tył nie testuje kinematyki

Przy `vx = 0` wszystkie koła dostają tę samą wartość, niezależnie od znaków przy
`vx`. Poprawność mieszania weryfikuje **wyłącznie jazda bokiem**. Test „jedzie do
przodu, czyli działa" jest bezwartościowy dla zmian w `drive()`.

## MAC-y i budowanie

Prawdziwe adresy leżą w `src/mac_addresses_private.h`, który jest w `.gitignore`.
`main.cpp` dołącza go warunkowo przez `__has_include`, a w razie braku sięga po
szablon `src/mac_addresses.h` z wyzerowanymi adresami. Dzięki temu CI i każdy
świeży klon budują się bez dostępu do prywatnych danych.

Nie usuwaj szablonu i nie commituj prywatnej kopii.

Budowanie: `pio run` (PlatformIO, środowisko `esp32-s3-devkitc-1`).
Każdy push i pull request jest sprawdzany przez GitHub Actions
(`.github/workflows/build.yml`).

## Znane luki

- **Brak failsafe.** Utrata łączności z Padem nie zatrzymuje robota — jedzie
  dalej z ostatnimi odebranymi wartościami joysticka. `Message_from_Pad.timeStamp`
  jest pod to przygotowany, ale nieużywany.
- `setTargetRPM()` kasuje stan `HardStopped`, a `motorControlTask` woła `drive()`
  bezwarunkowo co 20 ms — więc `hardStop()` zostałby anulowany w następnym cyklu.
  Do naprawy zanim pojawi się przycisk E-stop.
- Wejście z pada trafia do `drive()` jako surowe `int16_t`, bez skalowania do
  `MAX_RPM` i bez normalizacji, gdy suma składowych przekracza zakres.
- `dt` w PID jest w milisekundach, nie w sekundach — nastawy Ki/Kd nie są
  porównywalne z literaturą ani z zewnętrznymi narzędziami do strojenia.

## Jak weryfikować zmiany

Kompilacja i zielone CI mówią tylko tyle, że kod jest poprawny składniowo.
Zmiany w sterowaniu, kinematyce i PID weryfikuje się wgraniem na sprzęt i jazdą.
Przyjęty rytm pracy: jedna zamknięta zmiana → build → flash → sprawdzenie na
podłodze → dopiero następna zmiana.
