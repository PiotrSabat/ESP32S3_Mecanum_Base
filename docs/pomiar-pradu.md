# Pomiar prądu — co się nadaje, a co nie

Spisane 2026-08-27 na podstawie dokumentacji producentów. Pytanie wyjściowe:
czy INA226 zmierzy prąd silnika w zakresie −10…+10 A, skoro mostek H odwraca
polaryzację.

## Krótka odpowiedź

INA226 **jest dwukierunkowy**, więc znak nie jest przeszkodą. Przeszkodą jest
**zakres napięcia wspólnego**: 0…36 V, bez schodzenia poniżej masy. W przewodzie
silnika sterowanego mostkiem H potencjał skacze między masą a napięciem ogniw
z częstotliwością PWM, a w fazie zaniku prądu schodzi poniżej masy o spadek na
diodzie. **Do przewodu silnika INA226 się nie nadaje** — nadaje się do szyny
zasilania.

## INA226 (Texas Instruments)

| parametr | wartość |
|---|---|
| pomiar dwukierunkowy | tak |
| zakres napięcia na boczniku | **±81,92 mV** (pełna skala) |
| rozdzielczość | 16 bitów ze znakiem → **2,5 µV** na bit |
| napięcie wspólne (szyna) | **0…36 V**, nie schodzi poniżej masy |
| interfejs | I2C — bez własnego przetwornika |

Bocznik pod ±10 A: `81,92 mV / 10 A ≈ 8,2 mΩ`. Przy 2,5 µV na bit daje to
ok. **0,3 mA rozdzielczości** — grubo powyżej potrzeb.

Wniosek: **jeden układ na szynę zasilania** (albo po jednym na ogniwo) daje
prąd i napięcie po I2C, bez zajmowania przetwornika w mikrokontrolerze. To jest
akurat pomiar odpowiadający wprost na problem odcinania zabezpieczenia ogniw.

## Do pomiaru na silniku — dwie drogi

### ACS724LLCTR-10AB (Allegro), hall

| parametr | wartość |
|---|---|
| zakres | **−10…+10 A**, dokładnie ten pytany |
| zasada | hall, **izolacja galwaniczna do 2,4 kV** |
| rezystancja toru | ~1,2 mΩ |
| pasmo | 120 kHz |
| wyjście | analogowe, 200 mV/A, środek 2,5 V przy zasilaniu 5 V |

Izolacja sprawia, że **napięcie wspólne przestaje mieć znaczenie** — można go
wpiąć w dowolny punkt toru, także w przewód silnika. To jest odpowiedź na
pierwotne pytanie.

**Pułapka do sprawdzenia przed zakupem:** wyjście jest wyśrodkowane na 2,5 V
i sięga 0,5…4,5 V przy zasilaniu 5 V. Przetwornik ESP32-S3 pracuje do 3,3 V,
więc potrzebny jest dzielnik — a to zjada część rozdzielczości i dokłada błąd.

### INA240 (Texas Instruments), wzmacniacz do bocznika

Zaprojektowany wprost pod pomiar w przewodzie silnika: napięcie wspólne
**−4…+80 V** i **tłumienie zakłóceń od PWM**. Nieizolowany, wyjście analogowe.
Dokładniejszy od halla, ale wymaga bocznika i dobrego przetwornika.

## Zalecenie kolejności

1. **Najpierw jeden INA226 na szynę zasilania.** Odpowiada na realny problem
   (odcinanie zabezpieczenia), idzie po I2C, nie zajmuje przetwornika.
2. **Zmierzyć, ile prądu naprawdę bierze jedno koło**, zanim kupisz cztery
   czujniki. Rząd wielkości z zapisów: rozruch przy 5,61 A całości przechodził,
   7,40 A odcinał. Na koło wypada z tego znacznie mniej niż 10 A, a czujnik
   ±10 A użyty do prądów rzędu 2 A marnuje rozdzielczość — **±5 A może dać
   dwa razy lepszy pomiar**.
3. **Cztery czujniki na silniki dopiero z lepszym przetwornikiem.** Wyjście
   analogowe plus dzielnik plus nieliniowy przetwornik ESP32-S3 to trzy źródła
   błędu naraz.

## Źródła

- https://www.ti.com/product/INA226
- https://www.ti.com/lit/gpn/INA240
- https://www.pololu.com/product/4043 (ACS724LLCTR-10AB)
- https://www.allegromicro.com/-/media/files/datasheets/acs724-datasheet.ashx
