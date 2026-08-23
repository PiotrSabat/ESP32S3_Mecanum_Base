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

### Ograniczenie przyspieszenia: zjazd do zera musi zostać swobodny

`Motor::update()` prowadzi `_rampedTarget`, która goni `_targetRPM` z limitem
`MAX_ACCEL_RPM_PER_S`. Limit działa **tylko na oddalanie się od zera**. Ruch
zadanej w stronę zera przechodzi bez ograniczenia — i to nie jest przeoczenie:
gdyby ograniczyć symetrycznie, `softStop()` i hamowanie awaryjne po utracie
łączności wyhamowywałyby dwukrotnie dłużej. Kod wygląda na niepotrzebnie
zawiły i ktoś będzie chciał go „uprościć". Nie upraszczaj — to jest różnica
między zatrzymaniem w 320 ms a w 600 ms.

Samo ograniczenie istnieje, bo bez niego regulator przy skoku zadanej wystawia
w pierwszym cyklu pełne PWM, cztery silniki ruszają jak zwarcie i zabezpieczenie
prądowe ogniw odcina zasilanie. To się realnie zdarzyło po podniesieniu `Kp`.

### Moment przeciwny do obrotu musi być limitowany zależnie od prędkości

Gdy regulator podaje napięcie przeciwne do kierunku obrotu koła, prąd wynosi
`(U_baterii + SEM) / R` — czyli **więcej niż przy zwarciu**, bo siła
elektromotoryczna dodaje się zamiast odejmować. A SEM rośnie z prędkością.

Dlatego limit w `Motor::update()` maleje liniowo od `MAX_HOLDING_PWM` przy
zerowych obrotach do zera przy `MAX_RPM`. To nie jest ozdobnik:

- **przy stojącym kole** SEM ≈ 0, moment jest tani i platforma dzięki niemu
  trzyma pozycję pod naciskiem — bez tego koła dają się swobodnie obracać ręką;
- **przy pełnej prędkości** ten sam PWM oznacza prąd zrywający zabezpieczenie
  ogniw, więc limit schodzi do zera i hamowanie odbywa się wybiegiem.

Ustawienie stałego limitu w którąkolwiek stronę psuje jedno albo drugie —
oba przypadki wystąpiły realnie 2026-08-23 i oba były wyczuwalne na sprzęcie.

Powiązany warunek: wygaszanie na postoju (`_rampedTarget == 0` przy prędkości
poniżej `STANDSTILL_RPM`) zeruje wyjście i całkę. Próg musi zostać **wąski** —
przy szerszym zjada całkę także wtedy, gdy ktoś kręci kołem, i trzymanie
pozycji przestaje działać.

### Zabezpieczenie prądowe ogniw jest realnym ograniczeniem projektowym

Platforma ma zabezpieczenie prądowe na każdym z dwóch ogniw i ono **odcina
zasilanie w trakcie jazdy**, jeśli sterowanie zażąda za dużo prądu naraz.
Każda zmiana nastaw PID, limitów lub dynamiki musi być rozpatrzona pod kątem
prądu, a nie tylko prędkości i przeregulowania. Symulatory w `sim/` liczą prąd
właśnie po to.

Rząd wielkości do orientacji: rozruch przy 5,61 A przechodził bez odcięcia,
7,40 A odcinał.

### Nastawy PID mają nietypowe jednostki

`computePID()` dostaje `dt` w **milisekundach**, nie w sekundach. Dlatego
`Kd = 50` obok `Kp = 3` w `motor_config.h` nie jest literówką — w konwencji
sekundowej to `Kd = 0.05`, a `Ki = 0.03` to `Ki = 30`. Przeliczenie:
`Ki_ms = Ki_s / 1000`, `Kd_ms = Kd_s * 1000`.

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
- Pochodna liczona jest z **błędu**, nie z prędkości mierzonej, więc przy
  gwałtownej zmianie zadanej powstaje kopnięcie różniczkujące. Przy obecnym
  `Kd` jest ono realne; łagodzi je ograniczenie przyspieszenia, ale nie usuwa.
- `MAX_RPM = 180` jest wartością **założoną**, nie zmierzoną. Od niej zależy
  skalowanie drążka i normalizacja kinematyki. Jeśli w praktyce ostatni kawałek
  skoku drążka nic nie zmienia, wartość jest za wysoka; jeśli platforma nie
  wykorzystuje pełnej prędkości — za niska.
- Przekładnia 120:1 okazała się **odwracalna** — koło daje się obrócić ręką przy
  odciętym zasilaniu. Założenie „przekładnia sama zatrzyma platformę" jest więc
  prawdziwe tylko częściowo i hamowanie potrzebuje wsparcia od silnika.

## Jak weryfikować zmiany

Kompilacja i zielone CI mówią tylko tyle, że kod jest poprawny składniowo.
Zmiany w sterowaniu, kinematyce i PID weryfikuje się wgraniem na sprzęt i jazdą.
Przyjęty rytm pracy: jedna zamknięta zmiana → build → flash → sprawdzenie na
podłodze → dopiero następna zmiana.

Pośrednim krokiem są symulatory w `sim/` — uruchamiają prawdziwy `Motor.cpp`
i `MecanumDrive.cpp` na komputerze z podstawionym zegarem i enkoderem, więc
pokazują to, czego na jeżdżącej platformie nie widać: przebiegi w czasie, stany
wewnętrzne regulatora i pobierany prąd. Uruchomienie: `./sim/run.sh`.

Ważne ograniczenie: model napędu jest oszacowany, nie zmierzony. Wiarygodne są
**porównania** między wariantami, nie wartości bezwzględne. I model odpowiada
tylko na pytanie, które mu się zada — nastawy PID dobrane 2026-08-23 wyglądały
świetnie pod względem prędkości i wywaliły zabezpieczenie prądowe, bo ówczesny
symulator w ogóle nie liczył prądu.
