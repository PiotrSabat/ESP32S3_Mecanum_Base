# CLAUDE.md — Mecanum Platform (ESP32-S3)

Firmware czterokołowej platformy mecanum. Ten plik zawiera wyłącznie rzeczy,
których nie widać z samego kodu: trwałe niezmienniki i decyzje architektoniczne.
Świadomie **nie** ma tu statusu prac, planów ani nazw gałęzi — to rotuje
najszybciej i tworzy fałszywy kontekst.

## Architektura systemu

System składa się z **dwóch** urządzeń:

- **Pad** (Xiao ESP32-S3, repo `Pad_Adafruit_Xiao`, katalog lokalny
  `~/Documents/PlatformIO/Projects/Pad_I2C_ESP_32_S3` — nazwa katalogu nie
  odpowiada nazwie repo) — joysticki, wyświetlacz TFT.
  Pełni rolę pilota **oraz** monitora/debugera.
- **Platforma** (to repo) — cztery silniki DC z enkoderami, PID, kinematyka mecanum.

Osobny moduł „debug monitor" i aplikacja na iPhone zostały **porzucone**
(decyzja z 2026-08-23), żeby szybciej dowieźć działającą całość. Rolę monitora
przejął Pad, bo ma własny wyświetlacz i tak czy owak jest w rękach operatora.

Ślady po monitorze zostały usunięte: telemetria idzie do Pada, peer i adres
`macMonitorDebug` zniknęły, a `monitorUpdateTask` nazywa się `pidCommandTask`.
Została sama funkcja — zdalne nastawy PID (`MSG_SET_PID`) — której nadawcą
będzie Pad.

Komunikacja SPI została porzucona razem z monitorem. Jedynym kanałem jest ESP-NOW.

## Niezmienniki, których złamanie nie da błędu kompilacji

To są pułapki ciche — kod się zbuduje, CI przejdzie, a system nie zadziała.

### `src/messages.h` musi być identyczny z kopią w repo Pada

Plik definiuje protokół ESP-NOW i **oba repo muszą mieć go bajt w bajt takiego
samego**. Zmiana struktury to zmiana w obu repo w tym samym kroku.

Typ wiadomości rozpoznaje **pierwszy bajt** (`msgType`), a długość służy już
tylko do walidacji przed `memcpy`. Wcześniej typ był rozpoznawany po samym
`sizeof` — działało, dopóki rozmiary struktur były różne, a przy przypadkowej
zgodności dawało interpretację danych jako niewłaściwej struktury, po cichu.

Trzy rzeczy pilnują dziś tego niezmiennika:

- `static_assert` na rozmiarze każdej struktury — przypadkowa edycja w jednym
  repo staje się **błędem kompilacji**, a nie ciszą w eterze;
- `MSG_HELLO` wymieniane okresowo przez obie strony niesie `PROTO_VERSION`;
- **platforma nie ruszy**, dopóki nie zobaczy od Pada HELLO ze zgodną wersją
  (`padProtoOk` w warunku `linkAlive`). Odmowa jazdy jest tu funkcją
  bezpieczeństwa: gorsza od stania jest jazda na danych czytanych według
  cudzej wersji struktury.

HELLO są **powtarzane**, a nie uzgadniane raz na starcie, bo ESP-NOW nie zna
pojęcia sesji — każda strona może zniknąć i wrócić po resecie w dowolnej chwili.
Z tego samego powodu `msgType` leci w każdej ramce: pierwszy pakiet po restarcie
partnera musi być interpretowalny bez żadnej wcześniejszej wiedzy.

Pola prądu, pozycji i IMU w `Msg_Telemetry` są **zarezerwowane** — wypełniane
zerami do czasu, aż pojawią się boczniki i IMU. Są wśród nich `gyroYawRate`
i `accelX/accelY`, bo koła w poślizgu kręcą się szybciej, niż jedzie robot:
prędkość kątowa z żyroskopu porównana z tą policzoną z kół daje miarę poślizgu,
a przyspieszenia z kół nie da się odczytać w ogóle.

`targetRPM` i `measuredRPM` jadą w **konwencji robota** — dodatnie = do przodu
dla każdego koła. Odwrócenie prawej strony (`invertDirection`) jest odkręcane
w `telemetryTask`. Bez tego Pad musiałby znać okablowanie platformy, żeby
odwrócić mieszanie mecanum.

Echo osi (`echoAxis*`, `echoSeq`) wygląda na redundancję i nią nie jest.
Zgodna wersja protokołu dowodzi tylko, że obie strony mają ten sam plik —
**nie dowodzi, że platforma czyta właściwe pola**. Przesunięcie o dwa bajty
daje zielone „OK" i robota jadącego bokiem zamiast do przodu. Odesłanie osi
w postaci, w jakiej przyszły (przed martwą strefą i skalowaniem) zamyka tę
lukę, a Pad rysuje z tego kropkę w pierścieniu drążka: w środku = łącze żyje
i nadąża, wleczona = opóźnienie, skacząca = straty. Nie kasuj tych pól jako
„duplikatu danych z pada".

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

### Enkoder liczy ZLICZENIA, nie impulsy

Producent silnika podaje **8 impulsów na obrót wału**, czyli 960 na obrót koła
po przekładni 120:1. Ale `ESP32Encoder::attachHalfQuad` ustawia
`pos_mode = DEC` i `neg_mode = INC`, więc zlicza **oba zbocza** kanału A —
**dwa zliczenia na jeden impuls**. `DEFAULT_GEAR_RATIO` musi więc wynosić
**1920**, a nie 960.

Ten błąd siedział w repo od początku i **nie dawał żadnego objawu w kodzie**:
regulator poprawnie dochodził do zadanej, telemetria była spójna sama ze sobą,
symulatory przechodziły. Platforma po prostu jechała **dwa razy wolniej**, niż
twierdziła. Wyszło dopiero z zestawienia ekranu ze stoperem: 0,55 m/s na
wyświetlaczu, 3 m w 10,3 s na podłodze.

Wniosek na przyszłość: liczba impulsów w karcie katalogowej i liczba zliczeń
w bibliotece to **dwie różne rzeczy**, a pomyłka między nimi jest niewidoczna
dla wszystkiego poza pomiarem w świecie fizycznym.

### `motorControlTask` musi używać `vTaskDelayUntil`

`vTaskDelay` odmierza przerwę **od zakończenia pracy**, więc okres pętli wynosi
zadane 20 ms **plus czas wykonania**. Realnie dawało to 21 ms i regulator
pracował z innym `dt`, niż zakładają nastawy. Objaw był subtelny i wyszedł
dopiero z telemetrii: zmierzone obroty siadały na siatce kwantyzacji
przesuniętej względem właściwej.

Zamiana wygląda na kosmetykę, a przesuwa efektywne `Ki` i `Kd` o kilka procent.
Nie wracaj do `vTaskDelay` „dla uproszczenia".

### Prawy drążek steruje ciasnością łuku, nie prędkością obrotu

Wcześniej sterował wprost obrotem i miał tę samą władzę co drążek jazdy. Przy
pełnym gazie i pełnym obrocie wychodziło lewa 180 / prawa 0 — koła po
wewnętrznej **stały**, a platforma zaciągała zamiast wybierać łuk. Nie mogły
zacząć kontrować, bo normalizacja skaluje wszystkie cztery koła tym samym
współczynnikiem, a zero pozostaje zerem. Cały zakres łagodnych łuków ściskał się
w pierwszej jednej trzeciej skoku drążka.

`padAxisToOmega()` liczy człon obrotu **z prędkości jazdy**: przy pełnym gazie
lekki ruch drążka daje szeroki łuk, a pełne wychylenie ciasny zakręt z kołami
wewnętrznymi kontrującymi. Przy zatrzymanej platformie drążek wraca do roli
obrotu w miejscu — bez tego łuk o zerowej prędkości nie istnieje i nie dałoby
się obrócić w ogóle. Przejście między jednym a drugim jest płynne
(`PIVOT_BLEND_RPM`).

Wybrany świadomie wariant **terenowy**: ostry zakręt przy pełnej prędkości
pozostaje dostępny. Zmienia się rozkład czułości, a nie maksimum.

Pokrętłem do strojenia jest **`TURN_INNER_RATIO_FULL`**, a nie `TURN_GAIN` —
opisuje wprost to, co widać na podłodze: jaki ułamek prędkości kół zewnętrznych
mają koła wewnętrzne przy pełnym wychyleniu. Wartość ujemna oznacza kontrowanie.
`TURN_GAIN` jest z niej **wyliczane**, bo normalizacja dzieli przez `(1+G)`
i stosunek stron wychodzi `(1-G)/(1+G)`. Przy pierwszym podejściu wyszło −0,23
i wewnętrzne koła głównie hamowały; przy −0,55 obie strony pracują.

Ile z zadanej proporcji dojdzie do skutku **w chwili wchodzenia w zakręt**,
ogranicza limit momentu przeciwnego: koło toczące się jeszcze do przodu nie
dostanie pełnego rewersu, bo to dokładnie ten przypadek wywala zabezpieczenie
ogniw. To ograniczenie jest zamierzone i nie należy go obchodzić — w ustalonym
zakręcie, gdy koła wewnętrzne już się cofają, przestaje obowiązywać.

Prędkością do skalowania jest **długość wektora jazdy**, nie sama składowa
wzdłużna — przy mecanum jazda bokiem jest pełnoprawnym ruchem.

### Kształtowanie wejścia z drążków mieszka w `src/pad_input.h`

`padAxisToRPM()` i `padAxisToOmega()` są w osobnym nagłówku, bo korzystają
z nich **zarówno firmware, jak i symulatory**. Wcześniej symulator trzymał
własną kopię z komentarzem „ta sama arytmetyka" — czyli duplikat, który po
cichu się rozjeżdża i unieważnia testy, nie dając żadnego objawu.

Nagłówek celowo nie zależy od `Arduino.h`, żeby symulator budował się na
komputerze.

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
`Kd = 100` obok `Kp = 6` w `motor_config.h` nie jest literówką — w konwencji
sekundowej to `Kd = 0.1`, a `Ki = 0.06` to `Ki = 60`. Przeliczenie:
`Ki_ms = Ki_s / 1000`, `Kd_ms = Kd_s * 1000`.

Wzmocnienia zostały **podwojone** razem z poprawką liczby zliczeń enkodera.
Zmierzona prędkość i zadana zmalały wtedy dwukrotnie, więc błąd też — podwojenie
wzmocnień sprawia, że wyjście regulatora pozostaje **identyczne**. To nie było
strojenie, tylko zmiana jednostek. Z tego samego powodu przeskalowane zostały
`MAX_RPM` (180 → 90), `MAX_ACCEL_RPM_PER_S` (300 → 150) i `STANDSTILL_RPM`
(5 → 2,5). Limit momentu przeciwnego wychodzi na tym bez zmian, bo zależy od
ilorazu `prędkość / MAX_RPM`, a oba człony zmalały dwukrotnie.

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

W `.gitignore` obowiązuje wzorzec `*private*`, a nie konkretna nazwa pliku.
Konkretna nazwa nie chroni przed literówką, a literówka już raz wystąpiła:
`mac_adersses_private.cpp` trafił do historii z prawdziwymi adresami.

Budowanie: `pio run` (PlatformIO, środowisko `esp32-s3-devkitc-1`).
Każdy push i pull request jest sprawdzany przez GitHub Actions
(`.github/workflows/build.yml`).

Wersja platformy jest **przypięta**: `espressif32 @ ~7.0.1`. To nie jest
ostrożność na wyrost — 7.0.1 ciągnie Arduino core 2.0.17, a core 3.x **usunął**
`ledcSetup()` i `ledcAttachPin()`, których używa `Motor.cpp`. Bez przypięcia
build padłby kiedyś u wszystkich naraz, bez zmiany jednej linijki w kodzie.
Oba repo mają tę samą wersję, żeby nie rozjechały się w czasie.

## Język: kod po angielsku, ten plik po polsku

Komentarze w `src/` i `sim/`, README oraz `docs/` są **po angielsku** — repo
jest publiczne i ma trafić do ludzi spoza tego biurka. Ten plik zostaje po
polsku, bo jest roboczy i czyta go Piotr. Nie ujednolicaj tego w żadną stronę
bez pytania.

Ściągawki sprzętowe, żeby nie zgadywać: [docs/drivetrain.md](docs/drivetrain.md)
(koło, geometria, przeliczenia RPM) i
[docs/current-sensing.md](docs/current-sensing.md) (moduły do pomiaru prądu).

## Znane luki

- **Failsafe hamuje wybiegiem, a przekładnia okazała się odwracalna.** Po ciszy
  dłuższej niż `PAD_LINK_TIMEOUT_MS` `motorControlTask` woła `hardStop()` i
  przestaje wołać `drive()`. Komentarz przy tym kodzie uzasadnia wybieg tym, że
  „przekładnia 120:1 zatrzyma platformę sama" — a to założenie jest prawdziwe
  tylko częściowo (patrz punkt o odwracalności niżej). Nie sprawdzone na
  pochyłości; jeśli platforma odjeżdża, wybieg trzeba zastąpić `softStop()`.
- `setTargetRPM()` kasuje stan `HardStopped`. Dziś to nie szkodzi, bo jedyny
  `hardStop()` (failsafe) idzie w parze z zaprzestaniem wołania `drive()`.
  Wróci jako pułapka przy przycisku E-stop — wtedy stan musi być kasowany
  jawnie, a nie jako efekt uboczny zadania prędkości.
- `dt` w PID jest w milisekundach, nie w sekundach — nastawy Ki/Kd nie są
  porównywalne z literaturą ani z zewnętrznymi narzędziami do strojenia.
- Pochodna liczona jest z **błędu**, nie z prędkości mierzonej, więc przy
  gwałtownej zmianie zadanej powstaje kopnięcie różniczkujące. Przy obecnym
  `Kd` jest ono realne; łagodzi je ograniczenie przyspieszenia, ale nie usuwa.
- Przekładnia 120:1 okazała się **odwracalna** — koło daje się obrócić ręką przy
  odciętym zasilaniu. Założenie „przekładnia sama zatrzyma platformę" jest więc
  prawdziwe tylko częściowo i hamowanie potrzebuje wsparcia od silnika.

## Świadomie niezrobione (przegląd zewnętrzny 2026-08-28)

Rzeczy znalezione i **celowo** zostawione, z powodem. Lista odrzuconych
z uzasadnieniem jest warta więcej niż lista zrobionych.

- **`targetRPM` w telemetrii to zadana OPERATORA, nie zadana REGULATORA.**
  PID reguluje na `_rampedTarget` (po ograniczniku przyspieszenia), a w eter
  idzie `_targetRPM`. Przy każdym ruszaniu trójka target/measured/pwm pokazuje
  uchyb, którego regulator nie ma — wygląda to jak niedomagający PID, a jest
  ogranicznik działający zgodnie z projektem. Mylące dokładnie w chwili
  strojenia. Zmiana źródła to jedna metoda `getRampedTargetRPM()`; **czeka na
  decyzję Piotra**, bo to pytanie „co ma znaczyć ta liczba na ekranie", a nie
  pytanie techniczne. Argument za zmianą: pozycja drążka jest już widoczna jako
  kropka echa, więc intencja operatora ma drugie źródło, a regulator nie ma
  żadnego.
- **Anti-windup nie widzi ogranicznika momentu przeciwnego.** `computePID()`
  decyduje o przyjęciu całki na podstawie ±511, a docięcie przez `counterLimit`
  następuje już po jego wyjściu. Przy hamowaniu DO ZERA ratuje to wygaszanie na
  postoju (stąd zielony symulator `braking`), ale przy zjeździe do prędkości
  niezerowej (90 → 20 RPM) całka może się ładować w trakcie wybiegu.
  **Niezweryfikowane** — najpierw test w `sim/`, poprawka tylko jeśli test coś
  pokaże. `counterLimit` zależy wyłącznie od `_currentRPM`, znanego przed
  wywołaniem PID, więc dałoby się podać go jako efektywne granice cyklu.
- **`TFLAG_HARD_STOPPED` i `TFLAG_PWM_SAT` nigdy nie są ustawiane.** `Motor` nie
  wystawia stanu ani informacji o nasyceniu. Albo dorobić dwa gettery, albo
  usunąć bity — **usunięcie NIE wymaga podbicia `PROTO_VERSION`**, bo to stałe
  bitowe, a nie pola: układ struktury się nie zmienia.
- **`softStop()` jest nieosiągalny w firmware.** Wołany tylko w symulatorze;
  failsafe woła `hardStop()`. Lekarstwo na odwracalną przekładnię (punkt wyżej)
  jest więc napisane i przetestowane, tylko niepodłączone. Nie podłączać bez
  próby na pochyłości.
- **Pola protokołu bez konsumenta:** `rssiFromPad`, `rssiFromPlatform`,
  `platformLossPermille`, `motorCtrlTimeUs`, `mode`. Do podłączenia albo
  usunięcia, każde osobno. (`padLossPermille` i `flags` zostały podłączone.)
- **Prawdziwe adresy MAC zostają w historii gita.** Commit `748f7d8` dodał
  `src/mac_adersses_private.cpp` — z literówką w nazwie, więc `.gitignore` go
  nie złapał. Plik jest usunięty w HEAD, ale historia go pamięta. Ryzyko
  praktyczne bliskie zeru (to adresy rozgłaszane w eterze przy każdej ramce
  ESP-NOW), natomiast `git filter-repo` plus wymuszony push psuje istniejące
  klony. **Świadomie odpuszczone**; wzorzec `*private*` w `.gitignore` chroni
  przed powtórzeniem. Do zrobienia raz, w obu repo, jeśli kiedyś w ogóle.
- **Porządki na GitHubie:** PR #5 „Pr refactor pid v3" wisi otwarty od czerwca
  2025, a praca została w międzyczasie zastąpiona; trzy issues po polsku
  z grudnia 2024 i czerwca 2025, z czego dwa opisują kierunki właśnie
  porzucone. Zamknięte issue z jednozdaniowym powodem jest lepszą wizytówką
  niż otwarte bez. **Wymaga `gh` albo przeglądarki — nie da się zrobić
  z tego katalogu.**

## Jak weryfikować zmiany

Kompilacja i zielone CI mówią tylko tyle, że kod jest poprawny składniowo.
Zmiany w sterowaniu, kinematyce i PID weryfikuje się wgraniem na sprzęt i jazdą.
Przyjęty rytm pracy: jedna zamknięta zmiana → build → flash → sprawdzenie na
podłodze → dopiero następna zmiana.

Symulatory kompilują się z **`-Wall -Wextra`**, nie z `-w`. Poprzednia flaga
wyciszała wszystko; po włączeniu ostrzeżeń wyszły martwe zmienne i `-Wreorder`
w konstruktorze `Motor`. To jest tania klasa błędów do złapania — nie wracaj
do `-w` „dla czystszego wyjścia".

Pośrednim krokiem są symulatory w `sim/` — uruchamiają prawdziwy `Motor.cpp`
i `MecanumDrive.cpp` na komputerze z podstawionym zegarem i enkoderem, więc
pokazują to, czego na jeżdżącej platformie nie widać: przebiegi w czasie, stany
wewnętrzne regulatora i pobierany prąd. Uruchomienie: `./sim/run.sh`.

Ważne ograniczenie: model napędu jest oszacowany, nie zmierzony — choć od
2026-08-28 uwzględnia obciążenie (`I_LOAD`) i dzięki temu trafia w zmierzoną
prędkość maksymalną (~90 RPM) oraz w rząd wielkości prądu rozruchu (6 A wobec
zmierzonych 5,61 A przechodzących i 7,40 A odcinających). Wcześniej model
zaniżał prąd prawie czterokrotnie. Wiarygodne są nadal głównie **porównania**
między wariantami, nie wartości bezwzględne. I model odpowiada
tylko na pytanie, które mu się zada — nastawy PID dobrane 2026-08-23 wyglądały
świetnie pod względem prędkości i wywaliły zabezpieczenie prądowe, bo ówczesny
symulator w ogóle nie liczył prądu.
