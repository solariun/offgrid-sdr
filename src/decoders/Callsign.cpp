// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Callsign.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The offline geography of Callsign.h: a port of OFFgrid-SDR's
//  callCountry(), latLonToGrid(), gridLL(), distBrg() and of its tables
//  CTY_NAMES, CTY_GRID, CTY_LONG and CITIES (diddy-boy, index.html).  The
//  tables below are generated from the browser's (same order, same
//  indices; non-ASCII city names as UTF-8 octal escapes).
//
//  Additions to the JS (each marked where it is):
//    - Locator::fromLatLon() can give 4 characters;
//    - NaN / infinite positions give no locator, no distance;
//    - continentName() (the browser has the same words inline).
//
//  How it fits in
//  --------------
//  See Callsign.h.
//
//  Contributing
//  ------------
//  Regenerate the tables from index.html if the browser's change (a short
//  script reading the four JS constants as JSON does it).
// ============================================================================
#include "Callsign.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <vector>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;

bool finite(double v) { return v == v && v - v == 0; }

struct CountryName { const char* name; const char* cont; };
struct LongPrefix { const char* prefix; int index; };

// (generated from index.html's CTY_NAMES: 269 countries and areas, with their continents)
const CountryName kCtyNames[] = {
    {"Botswana", "AF"}, {"Tonga", "OC"}, {"Oman", "AS"},
    {"Bhutan", "AS"}, {"United Arab Emirates", "AS"}, {"Qatar", "AS"},
    {"Liberia", "AF"}, {"Bahrain", "AS"}, {"USA", "NA"},
    {"Spain", "EU"}, {"Pakistan", "AS"}, {"India", "AS"},
    {"Australia", "OC"}, {"Argentina", "SA"}, {"China", "AS"},
    {"Nauru", "OC"}, {"Andorra", "EU"}, {"Cyprus", "AS"},
    {"Gambia", "AF"}, {"Bahamas", "NA"}, {"Mozambique", "AF"},
    {"Chile", "SA"}, {"Canada", "NA"}, {"Cuba", "NA"},
    {"Morocco", "AF"}, {"Bolivia", "SA"}, {"Portugal", "EU"},
    {"Uruguay", "SA"}, {"Angola", "AF"}, {"Cape Verde", "AF"},
    {"Comoros", "AF"}, {"South Korea", "AS"}, {"Germany", "EU"},
    {"Philippines", "OC"}, {"Thailand", "AS"}, {"Eritrea", "AF"},
    {"Palestine", "AS"}, {"Cook Islands", "OC"}, {"Niue", "OC"},
    {"Bosnia and Herzegovina", "EU"}, {"Ireland", "EU"}, {"Armenia", "AS"},
    {"Ukraine", "EU"}, {"Iran", "AS"}, {"Moldova", "EU"},
    {"Estonia", "EU"}, {"Ethiopia", "AF"}, {"Belarus", "EU"},
    {"Kyrgyzstan", "AS"}, {"Tajikistan", "AS"}, {"Turkmenistan", "AS"},
    {"France", "EU"}, {"England", "EU"}, {"Panama", "NA"},
    {"Solomon Islands", "OC"}, {"Nicaragua", "NA"}, {"Hungary", "EU"},
    {"Switzerland", "EU"}, {"Ecuador", "SA"}, {"Poland", "EU"},
    {"Haiti", "NA"}, {"Dominican Republic", "NA"}, {"Colombia", "SA"},
    {"North Korea", "AS"}, {"Iraq", "AS"}, {"Honduras", "NA"},
    {"El Salvador", "NA"}, {"Vatican", "EU"}, {"Saudi Arabia", "AS"},
    {"Italy", "EU"}, {"Djibouti", "AF"}, {"Grenada", "NA"},
    {"Greece", "EU"}, {"Guinea-Bissau", "AF"}, {"St Lucia", "NA"},
    {"Dominica", "NA"}, {"St Vincent", "NA"}, {"Japan", "AS"},
    {"Mongolia", "AS"}, {"Norway", "EU"}, {"Jordan", "AS"},
    {"Indonesia", "OC"}, {"Luxembourg", "EU"}, {"Lithuania", "EU"},
    {"Bulgaria", "EU"}, {"Peru", "SA"}, {"Lebanon", "AS"},
    {"Austria", "EU"}, {"Finland", "EU"}, {"Czech Republic", "EU"},
    {"Slovakia", "EU"}, {"Belgium", "EU"}, {"Denmark", "EU"},
    {"Papua New Guinea", "OC"}, {"Aruba", "SA"}, {"Netherlands", "EU"},
    {"Netherlands Caribbean", "SA"}, {"Brazil", "SA"}, {"Suriname", "SA"},
    {"Russia", "EU"}, {"Bangladesh", "AS"}, {"Slovenia", "EU"},
    {"Singapore", "AS"}, {"Seychelles", "AF"}, {"South Africa", "AF"},
    {"S\303\243o Tom\303\251 and Pr\303\255ncipe", "AF"}, {"Sweden", "EU"}, {"Egypt", "AF"},
    {"Sudan", "AF"}, {"Tuvalu", "OC"}, {"Kiribati", "OC"},
    {"Somalia", "AF"}, {"Afghanistan", "AS"}, {"San Marino", "EU"},
    {"Palau", "OC"}, {"Turkey", "EU"}, {"Guatemala", "NA"},
    {"Costa Rica", "NA"}, {"Iceland", "EU"}, {"Cameroon", "AF"},
    {"Corsica", "EU"}, {"Central African Republic", "AF"}, {"Congo", "AF"},
    {"Gabon", "AF"}, {"Tunisia", "AF"}, {"Chad", "AF"},
    {"C\303\264te d'Ivoire", "AF"}, {"Benin", "AF"}, {"Mali", "AF"},
    {"Uzbekistan", "AS"}, {"Kazakhstan", "AS"}, {"Antigua and Barbuda", "NA"},
    {"Belize", "NA"}, {"St Kitts and Nevis", "NA"}, {"Namibia", "AF"},
    {"Micronesia", "OC"}, {"Marshall Islands", "OC"}, {"Brunei", "OC"},
    {"UK overseas", "NA"}, {"Hong Kong", "AS"}, {"UK overseas", "AS"},
    {"Mexico", "NA"}, {"Greenland", "NA"}, {"Burkina Faso", "AF"},
    {"Cambodia", "AS"}, {"Vietnam", "AS"}, {"Laos", "AS"},
    {"Macau", "AS"}, {"Myanmar", "AS"}, {"Vanuatu", "OC"},
    {"Syria", "AS"}, {"Latvia", "EU"}, {"Romania", "EU"},
    {"Serbia", "EU"}, {"Venezuela", "SA"}, {"Zimbabwe", "AF"},
    {"North Macedonia", "EU"}, {"Kosovo", "EU"}, {"South Sudan", "AF"},
    {"Albania", "EU"}, {"UK overseas", "EU"}, {"New Zealand", "OC"},
    {"UK overseas", "AF"}, {"Paraguay", "SA"}, {"Monaco", "EU"},
    {"Mauritius", "AF"}, {"Equatorial Guinea", "AF"}, {"Fiji", "OC"},
    {"Guinea", "AF"}, {"Bouvet Island", "AF"}, {"Azerbaijan", "AS"},
    {"Georgia", "AS"}, {"Montenegro", "EU"}, {"Sri Lanka", "AS"},
    {"United Nations", "EU"}, {"Timor-Leste", "OC"}, {"Israel", "AS"},
    {"Libya", "AF"}, {"Tanzania", "AF"}, {"Nigeria", "AF"},
    {"Madagascar", "AF"}, {"Mauritania", "AF"}, {"Niger", "AF"},
    {"Togo", "AF"}, {"Samoa", "OC"}, {"Uganda", "AF"},
    {"Kenya", "AF"}, {"Senegal", "AF"}, {"Jamaica", "NA"},
    {"Yemen", "AS"}, {"Lesotho", "AF"}, {"Malawi", "AF"},
    {"Algeria", "AF"}, {"Barbados", "NA"}, {"Maldives", "AS"},
    {"Guyana", "SA"}, {"Croatia", "EU"}, {"Ghana", "AF"},
    {"Malta", "EU"}, {"Zambia", "AF"}, {"Kuwait", "AS"},
    {"Sierra Leone", "AF"}, {"Malaysia", "AS"}, {"Nepal", "AS"},
    {"DR Congo", "AF"}, {"Burundi", "AF"}, {"Rwanda", "AF"},
    {"Trinidad and Tobago", "SA"}, {"Scotland", "EU"}, {"Wales", "EU"},
    {"Northern Ireland", "EU"}, {"Isle of Man", "EU"}, {"Jersey", "EU"},
    {"Guernsey", "EU"}, {"Hawaii", "OC"}, {"Alaska", "NA"},
    {"Puerto Rico", "NA"}, {"US Virgin Islands", "NA"}, {"Guam", "OC"},
    {"Mariana Islands", "OC"}, {"American Samoa", "OC"}, {"Balearic Islands", "EU"},
    {"Canary Islands", "AF"}, {"Ceuta and Melilla", "AF"}, {"Madeira", "AF"},
    {"Azores", "EU"}, {"Sardinia", "EU"}, {"Svalbard", "EU"},
    {"Jan Mayen", "EU"}, {"\303\205land Islands", "EU"}, {"Market Reef", "EU"},
    {"Faroe Islands", "EU"}, {"Guadeloupe", "NA"}, {"Martinique", "NA"},
    {"French Guiana", "SA"}, {"R\303\251union", "AF"}, {"New Caledonia", "OC"},
    {"French Polynesia", "OC"}, {"Saint Martin", "NA"}, {"Saint Barth\303\251lemy", "NA"},
    {"St Pierre and Miquelon", "NA"}, {"Mayotte", "AF"}, {"Wallis and Futuna", "OC"},
    {"Cura\303\247ao", "SA"}, {"Bonaire", "SA"}, {"Saba and St Eustatius", "NA"},
    {"Sint Maarten", "NA"}, {"Taiwan", "AS"}, {"Liechtenstein", "EU"},
    {"Gibraltar", "EU"}, {"UK bases on Cyprus", "AS"}, {"St Helena", "AF"},
    {"Ascension Island", "AF"}, {"Tristan da Cunha", "AF"}, {"Cayman Islands", "NA"},
    {"Anguilla", "NA"}, {"Montserrat", "NA"}, {"British Virgin Islands", "NA"},
    {"Turks and Caicos", "NA"}, {"Pitcairn Island", "OC"}, {"Falkland Islands", "SA"},
    {"Bermuda", "NA"}, {"Chagos Islands", "AF"}, {"Eswatini", "AF"},
    {"United Nations (Geneva)", "EU"}, {"United Nations (New York)", "NA"}, {"Australian islands", "OC"},
    {"Australian Antarctic", "AN"}, {"Antarctica", "AN"},
};
// (CTY_GRID: two characters for each pair of a call's first two characters, 36 x 36 pairs in
//  "0-9A-Z" order; each is a base-36 index into kCtyNames, "--" = no country)
const char kCtyGrid[] =
    "------------------------------------------------------------------------------------------------"
    "------------------------------------------------1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G"
    "1G1G1G1G1G1G1G1G1G1G1G1G--------------------4K4L4M4N1H1H0L0E0E0E0E0E0E0E0E0E0E0E0E0E0E3G414O4P1N"
    "--------------------3X3X3X0X0X0X0X0X0X4Q4Q4R4A494S4T4T4T4T2D4U1O4V4W4W4W--------------------4X0H"
    "0O0O0O0O0O4Y4Y1Q1Q06064Z4Z2K2K505051525354555656--------------------2Z2Z463X3X3X3X3X3X3X0V0V0V0V"
    "330A0A0A0A30305757505806--------------------2929292929292929292525252525595A5B5C2Y5C5C5C5C5C5C1W"
    "--------------------2929292929292929292525252525005D5E5F2Y0B0B0B0B0B0B1W--------------------5G17"
    "17171A1A5H5I5J5J5K5L5M5N5O5O5O5O5O5O5P2U5M5Q5R5R----00010203040506070808080808080808080808080909"
    "090A0A0A0A0B0B0B0B0C0D0D0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E0E"
    "----0F0G0H0I0J--0K0K0L0L0L0L0L0M0M0M0M0M0M0N0N0O0N0P0Q0Q0Q0Q0Q0R0R0R0M0M----0S0S0T060U0V0V0V0W0W"
    "0W0W0W0W0W0W0W0W0W0W0W0W0W0W0W0W0V0V0X0X0X0X0X0X----0Y0Z10111213----0909090909090909141415061616"
    "16171718191A1B1B1B1C1D1E1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F1F"
    "1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G----0H1H1I--1J1J1H1H1K1L"
    "1M1M1L1N1K1O1P1Q1Q0V1R1S1H1H1T1T0Y1J1U1V1F1F1F1W1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X1X"
    "1X1X1X1X1X1X1X1X1X1X1X1X----1Y1Z2021222324--2525252525252525252525252525252525252526262627272829"
    "080808080808080808080808080808080808080808080808080808080808080808080808----0D0D0D0D0D0D0D0D2727"
    "2727272727272727272727270D0D0D0D0D0D0D0D0D2A2B2C1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G1G"
    "1G1G1G1G1G1G1G1G1G1G1G1G080808080808080808080808080808080808080808080808080808080808080808080808"
    "--------------------2D2D2D2E2F2G2G2G2G2G2H2H2I2J2J2J2J2J2J2J2K2K2K2K2K2K----2L0H2M1R1R1R1R1R2N2N"
    "2N2N2N2N2N2N2N2O29292929292P2P2P2P2P2P2P2P2P2P2Q------------------------------------------------"
    "------------------------2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R2R"
    "----2S2S--2T2U2V2W2X2Y2Y2Y2Y2Y2Y2Y2Y2Y2Y2Y2Y2Y1N1N1N1N1N2Z302Z2020202020----31320N33343536--3737"
    "3738393A381F393B3C3D1F3E1F1F1F3F3G3H3I1F1F1F3J3K--------------------2R2R2R2R2R2R2R2R2R3L3L3L3L3M"
    "3M3M3M161616161616161616----3N3O3P3Q3R3S3T--0M0M0M0M0M0M0M0C0C0C0C0C0C0C0M3U3U3V3W0B0B0B0B0M0M0C"
    "080808080808080808080808080808080808080808080808080808080808080808080808--------------------3X3X"
    "3X3X3X3X3X3X3X0M0M0M0M0M0M3Y0L0L0E3Z404142434444----0W0W0W0W0W0W0W0W34292929292929291S454647371J"
    "484848481U49494A4A4A4A49----4B4C----4D--4E--4F4G4G4G4G4G4G4G4G4G4H4H4H4I4I4J4G2W2W2W2W2P2P2P2P2P"
    ;
// (CTY_LONG: longer prefixes, checked first, the longest first; index into kCtyNames)
const LongPrefix kCtyLong[] = {
    {"GM", 208}, {"MM", 208}, {"2M", 208}, {"GS", 208}, {"2S", 208}, {"GW", 209},
    {"MW", 209}, {"2W", 209}, {"GC", 209}, {"GI", 210}, {"MI", 210}, {"2I", 210},
    {"GN", 210}, {"GD", 211}, {"MD", 211}, {"2D", 211}, {"GT", 211}, {"GJ", 212},
    {"MJ", 212}, {"2J", 212}, {"GH", 212}, {"GU", 213}, {"MU", 213}, {"2U", 213},
    {"GP", 213}, {"KH6", 214}, {"WH6", 214}, {"NH6", 214}, {"AH6", 214}, {"KH7", 214},
    {"KL7", 215}, {"WL7", 215}, {"NL7", 215}, {"AL7", 215}, {"KP4", 216}, {"WP4", 216},
    {"NP4", 216}, {"KP3", 216}, {"KP2", 217}, {"NP2", 217}, {"WP2", 217}, {"KH2", 218},
    {"AH2", 218}, {"NH2", 218}, {"WH2", 218}, {"KH0", 219}, {"KH8", 220}, {"EA6", 221},
    {"EB6", 221}, {"EC6", 221}, {"ED6", 221}, {"EA8", 222}, {"EB8", 222}, {"EC8", 222},
    {"ED8", 222}, {"EF8", 222}, {"EA9", 223}, {"EB9", 223}, {"CT3", 224}, {"CQ3", 224},
    {"CR3", 224}, {"CU", 225}, {"CT8", 225}, {"IS0", 226}, {"IM0", 226}, {"IW0", 226},
    {"JW", 227}, {"JX", 228}, {"OH0", 229}, {"OJ0", 230}, {"OX", 142}, {"OY", 231},
    {"FG", 232}, {"FM", 233}, {"FY", 234}, {"FR", 235}, {"FK", 236}, {"FO", 237},
    {"FS", 238}, {"FJ", 239}, {"FP", 240}, {"FH", 241}, {"FW", 242}, {"PJ2", 243},
    {"PJ4", 244}, {"PJ5", 245}, {"PJ6", 245}, {"PJ7", 246}, {"BM", 247}, {"BN", 247},
    {"BO", 247}, {"BP", 247}, {"BQ", 247}, {"BU", 247}, {"BV", 247}, {"BW", 247},
    {"BX", 247}, {"HB0", 248}, {"HB3Y", 248}, {"ZB", 249}, {"ZC4", 250}, {"ZD7", 251},
    {"ZD8", 252}, {"ZD9", 253}, {"ZF", 254}, {"VP2E", 255}, {"VP2M", 256}, {"VP2V", 257},
    {"VP5", 258}, {"VQ5", 258}, {"VP6", 259}, {"VP8", 260}, {"VP9", 261}, {"VQ9", 262},
    {"3DA", 263}, {"SSN", 108}, {"SSO", 108}, {"SSP", 108}, {"SSQ", 108}, {"SSR", 108},
    {"SSS", 108}, {"SST", 108}, {"SSU", 108}, {"SSV", 108}, {"SSW", 108}, {"SSX", 108},
    {"SSY", 108}, {"SSZ", 108}, {"4U1I", 264}, {"4U1U", 265}, {"VK9", 266}, {"VK0", 267},
    {"KC4", 268}, {"CE9", 268}, {"DP0", 268},
};
// (CITIES: 620 cities: name (UTF-8), country, latitude, longitude)
const City kCities[] = {
    {"London", "UK", 51.51, -0.13},
    {"Birmingham", "UK", 52.49, -1.89},
    {"Manchester", "UK", 53.48, -2.24},
    {"Liverpool", "UK", 53.41, -2.98},
    {"Leeds", "UK", 53.8, -1.55},
    {"Sheffield", "UK", 53.38, -1.47},
    {"Bristol", "UK", 51.45, -2.59},
    {"Newcastle upon Tyne", "UK", 54.98, -1.61},
    {"Nottingham", "UK", 52.95, -1.15},
    {"Leicester", "UK", 52.64, -1.13},
    {"Coventry", "UK", 52.41, -1.51},
    {"Bradford", "UK", 53.79, -1.75},
    {"Hull", "UK", 53.74, -0.33},
    {"Stoke-on-Trent", "UK", 53.0, -2.18},
    {"Derby", "UK", 52.92, -1.48},
    {"Southampton", "UK", 50.9, -1.4},
    {"Portsmouth", "UK", 50.8, -1.09},
    {"Plymouth", "UK", 50.38, -4.14},
    {"Exeter", "UK", 50.72, -3.53},
    {"Bournemouth", "UK", 50.72, -1.88},
    {"Brighton", "UK", 50.82, -0.14},
    {"Norwich", "UK", 52.63, 1.3},
    {"Ipswich", "UK", 52.06, 1.16},
    {"Cambridge", "UK", 52.21, 0.12},
    {"Oxford", "UK", 51.75, -1.26},
    {"York", "UK", 53.96, -1.08},
    {"Lincoln", "UK", 53.23, -0.54},
    {"Middlesbrough", "UK", 54.57, -1.23},
    {"Carlisle", "UK", 54.89, -2.93},
    {"Lancaster", "UK", 54.05, -2.8},
    {"Preston", "UK", 53.76, -2.7},
    {"Blackpool", "UK", 53.82, -3.05},
    {"Truro", "UK", 50.26, -5.05},
    {"Penzance", "UK", 50.12, -5.54},
    {"Gloucester", "UK", 51.86, -2.24},
    {"Worcester", "UK", 52.19, -2.22},
    {"Hereford", "UK", 52.06, -2.72},
    {"Shrewsbury", "UK", 52.71, -2.75},
    {"Chester", "UK", 53.19, -2.89},
    {"Canterbury", "UK", 51.28, 1.08},
    {"Dover", "UK", 51.13, 1.31},
    {"Reading", "UK", 51.45, -0.97},
    {"Milton Keynes", "UK", 52.04, -0.76},
    {"Northampton", "UK", 52.24, -0.9},
    {"Peterborough", "UK", 52.57, -0.24},
    {"Swindon", "UK", 51.56, -1.78},
    {"Salisbury", "UK", 51.07, -1.79},
    {"Taunton", "UK", 51.02, -3.1},
    {"Cardiff", "UK", 51.48, -3.18},
    {"Swansea", "UK", 51.62, -3.94},
    {"Newport", "UK", 51.58, -2.99},
    {"Aberystwyth", "UK", 52.42, -4.08},
    {"Bangor", "UK", 53.23, -4.13},
    {"Wrexham", "UK", 53.05, -2.99},
    {"Edinburgh", "UK", 55.95, -3.19},
    {"Glasgow", "UK", 55.86, -4.25},
    {"Aberdeen", "UK", 57.15, -2.09},
    {"Dundee", "UK", 56.46, -2.97},
    {"Inverness", "UK", 57.48, -4.22},
    {"Perth", "UK", 56.4, -3.43},
    {"Stirling", "UK", 56.12, -3.94},
    {"Fort William", "UK", 56.82, -5.11},
    {"Oban", "UK", 56.42, -5.47},
    {"Thurso", "UK", 58.59, -3.52},
    {"Stornoway", "UK", 58.21, -6.39},
    {"Kirkwall", "UK", 58.98, -2.96},
    {"Lerwick", "UK", 60.15, -1.15},
    {"Dumfries", "UK", 55.07, -3.61},
    {"Belfast", "UK", 54.6, -5.93},
    {"Derry", "UK", 55.0, -7.31},
    {"Douglas", "Isle of Man", 54.15, -4.48},
    {"St Helier", "Jersey", 49.19, -2.11},
    {"St Peter Port", "Guernsey", 49.46, -2.54},
    {"Dublin", "Ireland", 53.35, -6.26},
    {"Cork", "Ireland", 51.9, -8.47},
    {"Galway", "Ireland", 53.27, -9.05},
    {"Limerick", "Ireland", 52.66, -8.63},
    {"Sligo", "Ireland", 54.27, -8.47},
    {"Paris", "France", 48.86, 2.35},
    {"Marseille", "France", 43.3, 5.37},
    {"Lyon", "France", 45.76, 4.84},
    {"Toulouse", "France", 43.6, 1.44},
    {"Nice", "France", 43.7, 7.27},
    {"Nantes", "France", 47.22, -1.55},
    {"Strasbourg", "France", 48.58, 7.75},
    {"Bordeaux", "France", 44.84, -0.58},
    {"Lille", "France", 50.63, 3.06},
    {"Rennes", "France", 48.11, -1.68},
    {"Brest", "France", 48.39, -4.49},
    {"Montpellier", "France", 43.61, 3.88},
    {"Ajaccio", "France", 41.93, 8.74},
    {"Berlin", "Germany", 52.52, 13.4},
    {"Hamburg", "Germany", 53.55, 9.99},
    {"Munich", "Germany", 48.14, 11.58},
    {"Cologne", "Germany", 50.94, 6.96},
    {"Frankfurt", "Germany", 50.11, 8.68},
    {"Stuttgart", "Germany", 48.78, 9.18},
    {"D\303\274sseldorf", "Germany", 51.23, 6.78},
    {"Leipzig", "Germany", 51.34, 12.37},
    {"Dresden", "Germany", 51.05, 13.74},
    {"Hanover", "Germany", 52.38, 9.73},
    {"Nuremberg", "Germany", 49.45, 11.08},
    {"Bremen", "Germany", 53.08, 8.8},
    {"Kiel", "Germany", 54.32, 10.13},
    {"Rostock", "Germany", 54.09, 12.1},
    {"Amsterdam", "Netherlands", 52.37, 4.9},
    {"Rotterdam", "Netherlands", 51.92, 4.48},
    {"The Hague", "Netherlands", 52.08, 4.3},
    {"Groningen", "Netherlands", 53.22, 6.57},
    {"Eindhoven", "Netherlands", 51.44, 5.47},
    {"Brussels", "Belgium", 50.85, 4.35},
    {"Antwerp", "Belgium", 51.22, 4.4},
    {"Ghent", "Belgium", 51.05, 3.72},
    {"Li\303\250ge", "Belgium", 50.63, 5.57},
    {"Luxembourg", "Luxembourg", 49.61, 6.13},
    {"Madrid", "Spain", 40.42, -3.7},
    {"Barcelona", "Spain", 41.39, 2.17},
    {"Valencia", "Spain", 39.47, -0.38},
    {"Seville", "Spain", 37.39, -5.98},
    {"Bilbao", "Spain", 43.26, -2.93},
    {"M\303\241laga", "Spain", 36.72, -4.42},
    {"Zaragoza", "Spain", 41.65, -0.89},
    {"A Coru\303\261a", "Spain", 43.36, -8.41},
    {"Palma", "Spain", 39.57, 2.65},
    {"Las Palmas", "Canary Islands", 28.12, -15.43},
    {"Santa Cruz de Tenerife", "Canary Islands", 28.46, -16.25},
    {"Lisbon", "Portugal", 38.72, -9.14},
    {"Porto", "Portugal", 41.15, -8.61},
    {"Faro", "Portugal", 37.02, -7.93},
    {"Funchal", "Madeira", 32.65, -16.91},
    {"Ponta Delgada", "Azores", 37.74, -25.67},
    {"Rome", "Italy", 41.9, 12.5},
    {"Milan", "Italy", 45.46, 9.19},
    {"Naples", "Italy", 40.85, 14.27},
    {"Turin", "Italy", 45.07, 7.69},
    {"Palermo", "Italy", 38.12, 13.36},
    {"Genoa", "Italy", 44.41, 8.93},
    {"Bologna", "Italy", 44.49, 11.34},
    {"Florence", "Italy", 43.77, 11.26},
    {"Venice", "Italy", 45.44, 12.32},
    {"Bari", "Italy", 41.12, 16.87},
    {"Cagliari", "Italy", 39.22, 9.12},
    {"Catania", "Italy", 37.5, 15.09},
    {"Bern", "Switzerland", 46.95, 7.45},
    {"Z\303\274rich", "Switzerland", 47.38, 8.54},
    {"Geneva", "Switzerland", 46.2, 6.14},
    {"Vienna", "Austria", 48.21, 16.37},
    {"Graz", "Austria", 47.07, 15.44},
    {"Innsbruck", "Austria", 47.27, 11.4},
    {"Salzburg", "Austria", 47.81, 13.04},
    {"Copenhagen", "Denmark", 55.68, 12.57},
    {"Aarhus", "Denmark", 56.16, 10.2},
    {"Oslo", "Norway", 59.91, 10.75},
    {"Bergen", "Norway", 60.39, 5.32},
    {"Trondheim", "Norway", 63.43, 10.4},
    {"Troms\303\270", "Norway", 69.65, 18.96},
    {"Stockholm", "Sweden", 59.33, 18.07},
    {"Gothenburg", "Sweden", 57.71, 11.97},
    {"Malm\303\266", "Sweden", 55.6, 13.0},
    {"Ume\303\245", "Sweden", 63.83, 20.26},
    {"Kiruna", "Sweden", 67.86, 20.23},
    {"Helsinki", "Finland", 60.17, 24.94},
    {"Tampere", "Finland", 61.5, 23.76},
    {"Oulu", "Finland", 65.01, 25.47},
    {"Rovaniemi", "Finland", 66.5, 25.73},
    {"Reykjav\303\255k", "Iceland", 64.15, -21.94},
    {"Akureyri", "Iceland", 65.68, -18.09},
    {"T\303\263rshavn", "Faroe Islands", 62.01, -6.77},
    {"Nuuk", "Greenland", 64.18, -51.72},
    {"Warsaw", "Poland", 52.23, 21.01},
    {"Krak\303\263w", "Poland", 50.06, 19.94},
    {"Gda\305\204sk", "Poland", 54.35, 18.65},
    {"Wroc\305\202aw", "Poland", 51.11, 17.03},
    {"Pozna\305\204", "Poland", 52.41, 16.93},
    {"Prague", "Czech Republic", 50.08, 14.44},
    {"Brno", "Czech Republic", 49.2, 16.61},
    {"Bratislava", "Slovakia", 48.15, 17.11},
    {"Budapest", "Hungary", 47.5, 19.04},
    {"Ljubljana", "Slovenia", 46.06, 14.51},
    {"Zagreb", "Croatia", 45.81, 15.98},
    {"Split", "Croatia", 43.51, 16.44},
    {"Sarajevo", "Bosnia and Herzegovina", 43.86, 18.41},
    {"Belgrade", "Serbia", 44.79, 20.45},
    {"Podgorica", "Montenegro", 42.44, 19.26},
    {"Pristina", "Kosovo", 42.66, 21.17},
    {"Skopje", "North Macedonia", 42.0, 21.43},
    {"Tirana", "Albania", 41.33, 19.82},
    {"Sofia", "Bulgaria", 42.7, 23.32},
    {"Varna", "Bulgaria", 43.21, 27.91},
    {"Bucharest", "Romania", 44.43, 26.1},
    {"Cluj-Napoca", "Romania", 46.77, 23.6},
    {"Chi\310\231in\304\203u", "Moldova", 47.01, 28.86},
    {"Athens", "Greece", 37.98, 23.73},
    {"Thessaloniki", "Greece", 40.64, 22.94},
    {"Heraklion", "Greece", 35.34, 25.13},
    {"Nicosia", "Cyprus", 35.17, 33.36},
    {"Valletta", "Malta", 35.9, 14.51},
    {"Monaco", "Monaco", 43.74, 7.42},
    {"Andorra la Vella", "Andorra", 42.51, 1.52},
    {"San Marino", "San Marino", 43.94, 12.45},
    {"Vaduz", "Liechtenstein", 47.14, 9.52},
    {"Tallinn", "Estonia", 59.44, 24.75},
    {"Riga", "Latvia", 56.95, 24.11},
    {"Vilnius", "Lithuania", 54.69, 25.28},
    {"Minsk", "Belarus", 53.9, 27.56},
    {"Kyiv", "Ukraine", 50.45, 30.52},
    {"Lviv", "Ukraine", 49.84, 24.03},
    {"Odesa", "Ukraine", 46.48, 30.73},
    {"Kharkiv", "Ukraine", 49.99, 36.23},
    {"Moscow", "Russia", 55.76, 37.62},
    {"Saint Petersburg", "Russia", 59.93, 30.34},
    {"Kaliningrad", "Russia", 54.71, 20.51},
    {"Murmansk", "Russia", 68.97, 33.08},
    {"Arkhangelsk", "Russia", 64.54, 40.54},
    {"Kazan", "Russia", 55.8, 49.11},
    {"Samara", "Russia", 53.2, 50.15},
    {"Volgograd", "Russia", 48.71, 44.51},
    {"Rostov-on-Don", "Russia", 47.24, 39.71},
    {"Sochi", "Russia", 43.6, 39.73},
    {"Yekaterinburg", "Russia", 56.84, 60.61},
    {"Perm", "Russia", 58.01, 56.25},
    {"Omsk", "Russia", 54.99, 73.37},
    {"Novosibirsk", "Russia", 55.01, 82.93},
    {"Krasnoyarsk", "Russia", 56.01, 92.87},
    {"Irkutsk", "Russia", 52.29, 104.3},
    {"Yakutsk", "Russia", 62.03, 129.73},
    {"Khabarovsk", "Russia", 48.48, 135.08},
    {"Vladivostok", "Russia", 43.12, 131.89},
    {"Magadan", "Russia", 59.57, 150.8},
    {"Petropavlovsk-Kamchatsky", "Russia", 53.02, 158.65},
    {"Norilsk", "Russia", 69.35, 88.2},
    {"Istanbul", "Turkey", 41.01, 28.98},
    {"Ankara", "Turkey", 39.93, 32.86},
    {"Izmir", "Turkey", 38.42, 27.14},
    {"Antalya", "Turkey", 36.9, 30.7},
    {"Tbilisi", "Georgia", 41.72, 44.79},
    {"Yerevan", "Armenia", 40.18, 44.51},
    {"Baku", "Azerbaijan", 40.41, 49.87},
    {"Cairo", "Egypt", 30.04, 31.24},
    {"Alexandria", "Egypt", 31.2, 29.92},
    {"Tripoli", "Libya", 32.89, 13.19},
    {"Tunis", "Tunisia", 36.81, 10.18},
    {"Algiers", "Algeria", 36.75, 3.06},
    {"Rabat", "Morocco", 34.02, -6.84},
    {"Casablanca", "Morocco", 33.57, -7.59},
    {"Marrakesh", "Morocco", 31.63, -8.01},
    {"Nouakchott", "Mauritania", 18.09, -15.98},
    {"Dakar", "Senegal", 14.72, -17.47},
    {"Banjul", "Gambia", 13.45, -16.58},
    {"Bissau", "Guinea-Bissau", 11.86, -15.6},
    {"Conakry", "Guinea", 9.64, -13.58},
    {"Freetown", "Sierra Leone", 8.47, -13.23},
    {"Monrovia", "Liberia", 6.3, -10.8},
    {"Abidjan", "C\303\264te d'Ivoire", 5.36, -4.01},
    {"Accra", "Ghana", 5.6, -0.19},
    {"Lom\303\251", "Togo", 6.13, 1.22},
    {"Porto-Novo", "Benin", 6.5, 2.6},
    {"Lagos", "Nigeria", 6.52, 3.38},
    {"Abuja", "Nigeria", 9.08, 7.4},
    {"Kano", "Nigeria", 12.0, 8.52},
    {"Niamey", "Niger", 13.51, 2.11},
    {"Ouagadougou", "Burkina Faso", 12.37, -1.52},
    {"Bamako", "Mali", 12.64, -8.0},
    {"N'Djamena", "Chad", 12.13, 15.06},
    {"Khartoum", "Sudan", 15.5, 32.56},
    {"Juba", "South Sudan", 4.85, 31.58},
    {"Addis Ababa", "Ethiopia", 9.03, 38.74},
    {"Asmara", "Eritrea", 15.32, 38.93},
    {"Djibouti", "Djibouti", 11.59, 43.15},
    {"Mogadishu", "Somalia", 2.05, 45.32},
    {"Nairobi", "Kenya", -1.29, 36.82},
    {"Mombasa", "Kenya", -4.04, 39.67},
    {"Kampala", "Uganda", 0.35, 32.58},
    {"Kigali", "Rwanda", -1.95, 30.06},
    {"Bujumbura", "Burundi", -3.38, 29.36},
    {"Dar es Salaam", "Tanzania", -6.79, 39.21},
    {"Dodoma", "Tanzania", -6.16, 35.75},
    {"Kinshasa", "DR Congo", -4.44, 15.27},
    {"Lubumbashi", "DR Congo", -11.66, 27.48},
    {"Brazzaville", "Congo", -4.26, 15.28},
    {"Libreville", "Gabon", 0.42, 9.47},
    {"Yaound\303\251", "Cameroon", 3.85, 11.5},
    {"Douala", "Cameroon", 4.05, 9.7},
    {"Malabo", "Equatorial Guinea", 3.75, 8.78},
    {"Bangui", "Central African Republic", 4.39, 18.56},
    {"Luanda", "Angola", -8.84, 13.23},
    {"Lusaka", "Zambia", -15.39, 28.32},
    {"Harare", "Zimbabwe", -17.83, 31.05},
    {"Lilongwe", "Malawi", -13.96, 33.79},
    {"Maputo", "Mozambique", -25.97, 32.57},
    {"Windhoek", "Namibia", -22.56, 17.08},
    {"Gaborone", "Botswana", -24.65, 25.91},
    {"Pretoria", "South Africa", -25.75, 28.19},
    {"Johannesburg", "South Africa", -26.2, 28.05},
    {"Cape Town", "South Africa", -33.92, 18.42},
    {"Durban", "South Africa", -29.86, 31.02},
    {"Port Elizabeth", "South Africa", -33.96, 25.6},
    {"Maseru", "Lesotho", -29.31, 27.48},
    {"Mbabane", "Eswatini", -26.31, 31.14},
    {"Antananarivo", "Madagascar", -18.88, 47.51},
    {"Port Louis", "Mauritius", -20.16, 57.5},
    {"Saint-Denis", "R\303\251union", -20.88, 55.45},
    {"Victoria", "Seychelles", -4.62, 55.45},
    {"Praia", "Cape Verde", 14.93, -23.51},
    {"Jerusalem", "Israel", 31.77, 35.21},
    {"Tel Aviv", "Israel", 32.09, 34.78},
    {"Amman", "Jordan", 31.95, 35.93},
    {"Beirut", "Lebanon", 33.89, 35.5},
    {"Damascus", "Syria", 33.51, 36.29},
    {"Baghdad", "Iraq", 33.31, 44.36},
    {"Basra", "Iraq", 30.51, 47.81},
    {"Kuwait City", "Kuwait", 29.38, 47.98},
    {"Riyadh", "Saudi Arabia", 24.71, 46.68},
    {"Jeddah", "Saudi Arabia", 21.49, 39.19},
    {"Manama", "Bahrain", 26.23, 50.59},
    {"Doha", "Qatar", 25.29, 51.53},
    {"Abu Dhabi", "United Arab Emirates", 24.45, 54.38},
    {"Dubai", "United Arab Emirates", 25.2, 55.27},
    {"Muscat", "Oman", 23.59, 58.41},
    {"Sana'a", "Yemen", 15.37, 44.19},
    {"Aden", "Yemen", 12.79, 45.02},
    {"Tehran", "Iran", 35.69, 51.39},
    {"Mashhad", "Iran", 36.3, 59.61},
    {"Shiraz", "Iran", 29.59, 52.58},
    {"Kabul", "Afghanistan", 34.53, 69.17},
    {"Islamabad", "Pakistan", 33.68, 73.05},
    {"Karachi", "Pakistan", 24.86, 67.0},
    {"Lahore", "Pakistan", 31.55, 74.34},
    {"New Delhi", "India", 28.61, 77.21},
    {"Mumbai", "India", 19.08, 72.88},
    {"Kolkata", "India", 22.57, 88.36},
    {"Chennai", "India", 13.08, 80.27},
    {"Bengaluru", "India", 12.97, 77.59},
    {"Hyderabad", "India", 17.39, 78.49},
    {"Ahmedabad", "India", 23.02, 72.57},
    {"Pune", "India", 18.52, 73.86},
    {"Jaipur", "India", 26.91, 75.79},
    {"Lucknow", "India", 26.85, 80.95},
    {"Guwahati", "India", 26.14, 91.74},
    {"Kathmandu", "Nepal", 27.72, 85.32},
    {"Thimphu", "Bhutan", 27.47, 89.64},
    {"Dhaka", "Bangladesh", 23.81, 90.41},
    {"Colombo", "Sri Lanka", 6.93, 79.86},
    {"Mal\303\251", "Maldives", 4.18, 73.51},
    {"Tashkent", "Uzbekistan", 41.3, 69.24},
    {"Samarkand", "Uzbekistan", 39.65, 66.96},
    {"Astana", "Kazakhstan", 51.17, 71.45},
    {"Almaty", "Kazakhstan", 43.24, 76.89},
    {"Bishkek", "Kyrgyzstan", 42.87, 74.59},
    {"Dushanbe", "Tajikistan", 38.56, 68.79},
    {"Ashgabat", "Turkmenistan", 37.96, 58.33},
    {"Ulaanbaatar", "Mongolia", 47.89, 106.91},
    {"Beijing", "China", 39.9, 116.41},
    {"Shanghai", "China", 31.23, 121.47},
    {"Guangzhou", "China", 23.13, 113.26},
    {"Shenzhen", "China", 22.54, 114.06},
    {"Chengdu", "China", 30.57, 104.07},
    {"Chongqing", "China", 29.56, 106.55},
    {"Wuhan", "China", 30.59, 114.31},
    {"Xi'an", "China", 34.34, 108.94},
    {"Harbin", "China", 45.8, 126.53},
    {"Shenyang", "China", 41.81, 123.43},
    {"Kunming", "China", 25.04, 102.71},
    {"Lhasa", "China", 29.65, 91.13},
    {"\303\234r\303\274mqi", "China", 43.83, 87.62},
    {"Hong Kong", "Hong Kong", 22.32, 114.17},
    {"Macau", "Macau", 22.2, 113.54},
    {"Taipei", "Taiwan", 25.03, 121.56},
    {"Kaohsiung", "Taiwan", 22.63, 120.3},
    {"Seoul", "South Korea", 37.57, 126.98},
    {"Busan", "South Korea", 35.18, 129.08},
    {"Pyongyang", "North Korea", 39.04, 125.76},
    {"Tokyo", "Japan", 35.68, 139.69},
    {"Osaka", "Japan", 34.69, 135.5},
    {"Nagoya", "Japan", 35.18, 136.91},
    {"Sapporo", "Japan", 43.06, 141.35},
    {"Fukuoka", "Japan", 33.59, 130.4},
    {"Sendai", "Japan", 38.27, 140.87},
    {"Hiroshima", "Japan", 34.39, 132.46},
    {"Naha", "Japan", 26.21, 127.68},
    {"Bangkok", "Thailand", 13.76, 100.5},
    {"Chiang Mai", "Thailand", 18.79, 98.99},
    {"Phuket", "Thailand", 7.88, 98.39},
    {"Hanoi", "Vietnam", 21.03, 105.85},
    {"Ho Chi Minh City", "Vietnam", 10.82, 106.63},
    {"Vientiane", "Laos", 17.98, 102.63},
    {"Phnom Penh", "Cambodia", 11.56, 104.92},
    {"Yangon", "Myanmar", 16.87, 96.2},
    {"Naypyidaw", "Myanmar", 19.76, 96.08},
    {"Kuala Lumpur", "Malaysia", 3.14, 101.69},
    {"Kota Kinabalu", "Malaysia", 5.98, 116.07},
    {"Kuching", "Malaysia", 1.55, 110.36},
    {"Singapore", "Singapore", 1.35, 103.82},
    {"Bandar Seri Begawan", "Brunei", 4.9, 114.94},
    {"Jakarta", "Indonesia", -6.21, 106.85},
    {"Surabaya", "Indonesia", -7.25, 112.75},
    {"Medan", "Indonesia", 3.6, 98.67},
    {"Denpasar", "Indonesia", -8.65, 115.22},
    {"Makassar", "Indonesia", -5.15, 119.43},
    {"Jayapura", "Indonesia", -2.53, 140.72},
    {"Dili", "Timor-Leste", -8.56, 125.57},
    {"Manila", "Philippines", 14.6, 120.98},
    {"Cebu", "Philippines", 10.32, 123.89},
    {"Davao", "Philippines", 7.19, 125.46},
    {"Sydney", "Australia", -33.87, 151.21},
    {"Melbourne", "Australia", -37.81, 144.96},
    {"Brisbane", "Australia", -27.47, 153.03},
    {"Perth", "Australia", -31.95, 115.86},
    {"Adelaide", "Australia", -34.93, 138.6},
    {"Canberra", "Australia", -35.28, 149.13},
    {"Hobart", "Australia", -42.88, 147.33},
    {"Darwin", "Australia", -12.46, 130.84},
    {"Cairns", "Australia", -16.92, 145.77},
    {"Townsville", "Australia", -19.26, 146.82},
    {"Alice Springs", "Australia", -23.7, 133.88},
    {"Broome", "Australia", -17.96, 122.24},
    {"Kalgoorlie", "Australia", -30.75, 121.47},
    {"Auckland", "New Zealand", -36.85, 174.76},
    {"Wellington", "New Zealand", -41.29, 174.78},
    {"Christchurch", "New Zealand", -43.53, 172.64},
    {"Dunedin", "New Zealand", -45.88, 170.5},
    {"Port Moresby", "Papua New Guinea", -9.44, 147.18},
    {"Suva", "Fiji", -18.14, 178.44},
    {"Noum\303\251a", "New Caledonia", -22.28, 166.46},
    {"Port Vila", "Vanuatu", -17.73, 168.32},
    {"Honiara", "Solomon Islands", -9.43, 159.95},
    {"Apia", "Samoa", -13.83, -171.76},
    {"Nuku\312\273alofa", "Tonga", -21.14, -175.2},
    {"Papeete", "French Polynesia", -17.54, -149.57},
    {"Tarawa", "Kiribati", 1.45, 173.03},
    {"Majuro", "Marshall Islands", 7.09, 171.38},
    {"Hag\303\245t\303\261a", "Guam", 13.48, 144.75},
    {"Honolulu", "Hawaii", 21.31, -157.86},
    {"Hilo", "Hawaii", 19.72, -155.08},
    {"Ottawa", "Canada", 45.42, -75.7},
    {"Toronto", "Canada", 43.65, -79.38},
    {"Montreal", "Canada", 45.5, -73.57},
    {"Quebec City", "Canada", 46.81, -71.21},
    {"Halifax", "Canada", 44.65, -63.58},
    {"St. John's", "Canada", 47.56, -52.71},
    {"Fredericton", "Canada", 45.96, -66.64},
    {"Charlottetown", "Canada", 46.24, -63.13},
    {"Winnipeg", "Canada", 49.9, -97.14},
    {"Regina", "Canada", 50.45, -104.62},
    {"Saskatoon", "Canada", 52.13, -106.67},
    {"Calgary", "Canada", 51.05, -114.07},
    {"Edmonton", "Canada", 53.55, -113.49},
    {"Vancouver", "Canada", 49.28, -123.12},
    {"Victoria", "Canada", 48.43, -123.37},
    {"Whitehorse", "Canada", 60.72, -135.06},
    {"Yellowknife", "Canada", 62.45, -114.37},
    {"Iqaluit", "Canada", 63.75, -68.52},
    {"Thunder Bay", "Canada", 48.38, -89.25},
    {"Sudbury", "Canada", 46.49, -80.99},
    {"Washington DC", "USA", 38.91, -77.04},
    {"New York", "USA", 40.71, -73.99},
    {"Boston", "USA", 42.36, -71.06},
    {"Philadelphia", "USA", 39.95, -75.17},
    {"Baltimore", "USA", 39.29, -76.61},
    {"Pittsburgh", "USA", 40.44, -80.0},
    {"Buffalo", "USA", 42.89, -78.88},
    {"Albany", "USA", 42.65, -73.76},
    {"Hartford", "USA", 41.76, -72.67},
    {"Providence", "USA", 41.82, -71.41},
    {"Portland (Maine)", "USA", 43.66, -70.26},
    {"Burlington", "USA", 44.48, -73.21},
    {"Concord", "USA", 43.21, -71.54},
    {"Newark", "USA", 40.74, -74.17},
    {"Richmond", "USA", 37.54, -77.44},
    {"Norfolk", "USA", 36.85, -76.29},
    {"Raleigh", "USA", 35.78, -78.64},
    {"Charlotte", "USA", 35.23, -80.84},
    {"Charleston (SC)", "USA", 32.78, -79.93},
    {"Columbia", "USA", 34.0, -81.03},
    {"Atlanta", "USA", 33.75, -84.39},
    {"Savannah", "USA", 32.08, -81.09},
    {"Jacksonville", "USA", 30.33, -81.66},
    {"Orlando", "USA", 28.54, -81.38},
    {"Tampa", "USA", 27.95, -82.46},
    {"Miami", "USA", 25.76, -80.19},
    {"Key West", "USA", 24.56, -81.78},
    {"Tallahassee", "USA", 30.44, -84.28},
    {"Birmingham (Alabama)", "USA", 33.52, -86.8},
    {"Montgomery", "USA", 32.37, -86.3},
    {"Mobile", "USA", 30.69, -88.04},
    {"Jackson", "USA", 32.3, -90.18},
    {"New Orleans", "USA", 29.95, -90.07},
    {"Baton Rouge", "USA", 30.45, -91.15},
    {"Memphis", "USA", 35.15, -90.05},
    {"Nashville", "USA", 36.16, -86.78},
    {"Knoxville", "USA", 35.96, -83.92},
    {"Louisville", "USA", 38.25, -85.76},
    {"Lexington", "USA", 38.04, -84.5},
    {"Cincinnati", "USA", 39.1, -84.51},
    {"Columbus", "USA", 39.96, -83.0},
    {"Cleveland", "USA", 41.5, -81.69},
    {"Detroit", "USA", 42.33, -83.05},
    {"Grand Rapids", "USA", 42.96, -85.67},
    {"Indianapolis", "USA", 39.77, -86.16},
    {"Chicago", "USA", 41.88, -87.63},
    {"Milwaukee", "USA", 43.04, -87.91},
    {"Madison", "USA", 43.07, -89.4},
    {"Minneapolis", "USA", 44.98, -93.27},
    {"Duluth", "USA", 46.79, -92.1},
    {"Des Moines", "USA", 41.59, -93.62},
    {"St. Louis", "USA", 38.63, -90.2},
    {"Kansas City", "USA", 39.1, -94.58},
    {"Omaha", "USA", 41.26, -95.93},
    {"Wichita", "USA", 37.69, -97.34},
    {"Oklahoma City", "USA", 35.47, -97.52},
    {"Tulsa", "USA", 36.15, -95.99},
    {"Little Rock", "USA", 34.75, -92.29},
    {"Dallas", "USA", 32.78, -96.8},
    {"Fort Worth", "USA", 32.76, -97.33},
    {"Houston", "USA", 29.76, -95.37},
    {"San Antonio", "USA", 29.42, -98.49},
    {"Austin", "USA", 30.27, -97.74},
    {"El Paso", "USA", 31.76, -106.49},
    {"Amarillo", "USA", 35.22, -101.83},
    {"Albuquerque", "USA", 35.08, -106.65},
    {"Santa Fe", "USA", 35.69, -105.94},
    {"Denver", "USA", 39.74, -104.99},
    {"Colorado Springs", "USA", 38.83, -104.82},
    {"Cheyenne", "USA", 41.14, -104.82},
    {"Fargo", "USA", 46.88, -96.79},
    {"Bismarck", "USA", 46.81, -100.78},
    {"Sioux Falls", "USA", 43.55, -96.73},
    {"Rapid City", "USA", 44.08, -103.23},
    {"Billings", "USA", 45.78, -108.5},
    {"Helena", "USA", 46.59, -112.04},
    {"Boise", "USA", 43.62, -116.2},
    {"Salt Lake City", "USA", 40.76, -111.89},
    {"Phoenix", "USA", 33.45, -112.07},
    {"Tucson", "USA", 32.22, -110.97},
    {"Las Vegas", "USA", 36.17, -115.14},
    {"Reno", "USA", 39.53, -119.81},
    {"Los Angeles", "USA", 34.05, -118.24},
    {"San Diego", "USA", 32.72, -117.16},
    {"San Francisco", "USA", 37.77, -122.42},
    {"San Jose", "USA", 37.34, -121.89},
    {"Sacramento", "USA", 38.58, -121.49},
    {"Fresno", "USA", 36.74, -119.79},
    {"Portland (Oregon)", "USA", 45.52, -122.68},
    {"Eugene", "USA", 44.05, -123.09},
    {"Seattle", "USA", 47.61, -122.33},
    {"Spokane", "USA", 47.66, -117.43},
    {"Anchorage", "Alaska", 61.22, -149.9},
    {"Fairbanks", "Alaska", 64.84, -147.72},
    {"Juneau", "Alaska", 58.3, -134.42},
    {"Nome", "Alaska", 64.5, -165.41},
    {"San Juan", "Puerto Rico", 18.47, -66.11},
    {"Mexico City", "Mexico", 19.43, -99.13},
    {"Guadalajara", "Mexico", 20.66, -103.35},
    {"Monterrey", "Mexico", 25.69, -100.32},
    {"Tijuana", "Mexico", 32.51, -117.04},
    {"Canc\303\272n", "Mexico", 21.16, -86.85},
    {"M\303\251rida", "Mexico", 20.97, -89.62},
    {"Oaxaca", "Mexico", 17.07, -96.73},
    {"Chihuahua", "Mexico", 28.63, -106.07},
    {"Guatemala City", "Guatemala", 14.63, -90.51},
    {"Belize City", "Belize", 17.5, -88.2},
    {"San Salvador", "El Salvador", 13.69, -89.22},
    {"Tegucigalpa", "Honduras", 14.07, -87.19},
    {"Managua", "Nicaragua", 12.11, -86.24},
    {"San Jos\303\251", "Costa Rica", 9.93, -84.08},
    {"Panama City", "Panama", 8.98, -79.52},
    {"Havana", "Cuba", 23.11, -82.37},
    {"Santiago de Cuba", "Cuba", 20.02, -75.82},
    {"Kingston", "Jamaica", 17.97, -76.79},
    {"Port-au-Prince", "Haiti", 18.59, -72.31},
    {"Santo Domingo", "Dominican Republic", 18.49, -69.93},
    {"Nassau", "Bahamas", 25.05, -77.36},
    {"Hamilton", "Bermuda", 32.29, -64.78},
    {"Bridgetown", "Barbados", 13.1, -59.62},
    {"Port of Spain", "Trinidad and Tobago", 10.65, -61.52},
    {"Willemstad", "Cura\303\247ao", 12.11, -68.93},
    {"Oranjestad", "Aruba", 12.52, -70.03},
    {"Fort-de-France", "Martinique", 14.62, -61.06},
    {"Pointe-\303\240-Pitre", "Guadeloupe", 16.24, -61.53},
    {"Bogot\303\241", "Colombia", 4.71, -74.07},
    {"Medell\303\255n", "Colombia", 6.24, -75.58},
    {"Cali", "Colombia", 3.45, -76.53},
    {"Caracas", "Venezuela", 10.48, -66.9},
    {"Maracaibo", "Venezuela", 10.65, -71.64},
    {"Georgetown", "Guyana", 6.8, -58.16},
    {"Paramaribo", "Suriname", 5.85, -55.2},
    {"Cayenne", "French Guiana", 4.92, -52.31},
    {"Quito", "Ecuador", -0.18, -78.47},
    {"Guayaquil", "Ecuador", -2.17, -79.92},
    {"Lima", "Peru", -12.05, -77.04},
    {"Cusco", "Peru", -13.53, -71.97},
    {"La Paz", "Bolivia", -16.49, -68.12},
    {"Santa Cruz", "Bolivia", -17.81, -63.16},
    {"Asunci\303\263n", "Paraguay", -25.26, -57.58},
    {"Montevideo", "Uruguay", -34.9, -56.16},
    {"Buenos Aires", "Argentina", -34.6, -58.38},
    {"C\303\263rdoba", "Argentina", -31.42, -64.18},
    {"Rosario", "Argentina", -32.95, -60.64},
    {"Mendoza", "Argentina", -32.89, -68.83},
    {"Salta", "Argentina", -24.78, -65.41},
    {"Bariloche", "Argentina", -41.13, -71.31},
    {"Ushuaia", "Argentina", -54.8, -68.3},
    {"Santiago", "Chile", -33.45, -70.67},
    {"Valpara\303\255so", "Chile", -33.05, -71.62},
    {"Antofagasta", "Chile", -23.65, -70.4},
    {"Punta Arenas", "Chile", -53.16, -70.91},
    {"Stanley", "Falkland Islands", -51.7, -57.85},
    {"Bras\303\255lia", "Brazil", -15.79, -47.88},
    {"S\303\243o Paulo", "Brazil", -23.55, -46.63},
    {"Rio de Janeiro", "Brazil", -22.91, -43.17},
    {"Belo Horizonte", "Brazil", -19.92, -43.94},
    {"Salvador", "Brazil", -12.97, -38.5},
    {"Fortaleza", "Brazil", -3.73, -38.53},
    {"Recife", "Brazil", -8.05, -34.88},
    {"Manaus", "Brazil", -3.12, -60.02},
    {"Bel\303\251m", "Brazil", -1.46, -48.49},
    {"Curitiba", "Brazil", -25.43, -49.27},
    {"Porto Alegre", "Brazil", -30.03, -51.23},
    {"Cuiab\303\241", "Brazil", -15.6, -56.1},
    {"Natal", "Brazil", -5.79, -35.21},
};

}  // namespace

namespace {

const char kCtyCh[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const int kNames = static_cast<int>(sizeof(kCtyNames) / sizeof(kCtyNames[0]));

int chIndex(char c) {
    const char* p = c ? std::strchr(kCtyCh, c) : 0;
    return p ? static_cast<int>(p - kCtyCh) : -1;
}

std::string upper(const std::string& s) {
    std::string r(s);
    for (std::size_t i = 0; i < r.size(); ++i) r[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(r[i])));
    return r;
}

std::string lower(const std::string& s) {
    std::string r(s);
    for (std::size_t i = 0; i < r.size(); ++i) r[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(r[i])));
    return r;
}

// the JS test /^(P|M|QRP|R|A|LH|\d)$/: a suffix that is not a prefix
bool isPlainSuffix(const std::string& p) {
    return p == "P" || p == "M" || p == "QRP" || p == "R" || p == "A" || p == "LH" ||
           (p.size() == 1 && std::isdigit(static_cast<unsigned char>(p[0])));
}

Country makeCountry(int i) {
    Country c;
    if (i < 0 || i >= kNames) return c;
    c.found = true;
    c.name = kCtyNames[i].name;
    c.continent = kCtyNames[i].cont;
    return c;
}

}  // namespace

Country countryOf(const std::string& call) {
    if (call.empty() || call[0] == '<') return Country();
    // a portable or visiting call (EA8/G4ABC, G4ABC/P, W1AW/4): the part that is a prefix decides; /P, /M,
    // /MM, /QRP and the like don't, and /MM (at sea) has no country
    std::string c = upper(call);
    std::vector<std::string> parts;
    {
        std::size_t a = 0;
        for (;;) {
            std::size_t b = c.find('/', a);
            parts.push_back(c.substr(a, b == std::string::npos ? std::string::npos : b - a));
            if (b == std::string::npos) break;
            a = b + 1;
        }
    }
    if (parts.size() > 1) {
        for (std::size_t i = 0; i < parts.size(); ++i)
            if (parts[i] == "MM" || parts[i] == "AM") return Country();
        int pre = -1;
        for (std::size_t i = 0; i < parts.size() && pre < 0; ++i) {
            const std::string& p = parts[i];
            if (p.size() < 1 || p.size() > 4 || isPlainSuffix(p)) continue;
            for (std::size_t j = 0; j < parts.size(); ++j)
                if (j != i && parts[j].size() > p.size()) { pre = static_cast<int>(i); break; }
        }
        if (pre >= 0) c = parts[pre];
        else {
            std::size_t best = 0;
            for (std::size_t i = 1; i < parts.size(); ++i) if (parts[i].size() > parts[best].size()) best = i;
            c = parts[best];
        }
    }
    for (std::size_t n = c.size() < 4 ? c.size() : 4; n >= 2; --n) {
        const std::string pre = c.substr(0, n);
        for (std::size_t i = 0; i < sizeof(kCtyLong) / sizeof(kCtyLong[0]); ++i)
            if (pre == kCtyLong[i].prefix) return makeCountry(kCtyLong[i].index);
    }
    if (c.size() < 2 || chIndex(c[0]) < 0 || chIndex(c[1]) < 0) return Country();
    // Russia east of the Urals: R or U(A-I) with call area 8, 9 or 0 is Asiatic Russia; area 2 is Kaliningrad
    {
        char d = 0;
        if (c[0] == 'R') {
            if (std::isdigit(static_cast<unsigned char>(c[1]))) d = c[1];
            else if (c[1] >= 'A' && c[1] <= 'Z' && c.size() > 2 && std::isdigit(static_cast<unsigned char>(c[2]))) d = c[2];
        } else if (c[0] == 'U' && c[1] >= 'A' && c[1] <= 'I' && c.size() > 2 && std::isdigit(static_cast<unsigned char>(c[2]))) {
            d = c[2];
        }
        if (d == '8' || d == '9' || d == '0') {
            Country r; r.found = true; r.name = "Asiatic Russia"; r.continent = "AS"; return r;
        }
        if (d == '2') {
            Country r; r.found = true; r.name = "Kaliningrad"; r.continent = "EU"; return r;
        }
    }
    const std::size_t k = static_cast<std::size_t>((chIndex(c[0]) * 36 + chIndex(c[1])) * 2);
    if (k + 1 >= sizeof(kCtyGrid)) return Country();
    const char c0 = kCtyGrid[k], c1 = kCtyGrid[k + 1];
    if (c0 == '-' || chIndex(c0) < 0 || chIndex(c1) < 0) return Country();
    return makeCountry(chIndex(c0) * 36 + chIndex(c1));
}

const char* continentName(const std::string& code) {
    // (Addition to the JS: the browser has these words inline in its FT8 box)
    if (code == "EU") return "Europe";
    if (code == "NA") return "North America";
    if (code == "SA") return "South America";
    if (code == "AS") return "Asia";
    if (code == "AF") return "Africa";
    if (code == "OC") return "Oceania";
    if (code == "AN") return "Antarctica";
    return "";
}

bool Locator::isValid(const std::string& g0) {
    const std::string g = upper(g0);
    if (g.size() != 4 && g.size() != 6) return false;
    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R') return false;
    if (!std::isdigit(static_cast<unsigned char>(g[2])) || !std::isdigit(static_cast<unsigned char>(g[3]))) return false;
    if (g.size() == 6 && (g[4] < 'A' || g[4] > 'X' || g[5] < 'A' || g[5] > 'X')) return false;
    return true;
}

bool Locator::toLatLon(const std::string& g0, double& lat, double& lon) {
    if (!isValid(g0)) return false;
    const std::string g = upper(g0);
    lon = (g[0] - 'A') * 20 - 180 + (g[2] - '0') * 2;
    lat = (g[1] - 'A') * 10 - 90 + (g[3] - '0');
    if (g.size() == 6) {
        lon += (g[4] - 'A') * (2.0 / 24) + 1.0 / 24;
        lat += (g[5] - 'A') * (1.0 / 24) + 0.5 / 24;
    } else {
        lon += 1;
        lat += 0.5;
    }
    return true;
}

std::string Locator::fromLatLon(double lat, double lon, int chars) {
    if (!finite(lat) || !finite(lon)) return std::string();   // (Addition to the JS)
    double x = lon + 180, y = lat + 90;
    x = std::min(359.9999, std::max(0.0, x));
    y = std::min(179.9999, std::max(0.0, y));
    std::string r;
    r += static_cast<char>('A' + static_cast<int>(std::floor(x / 20)));
    r += static_cast<char>('A' + static_cast<int>(std::floor(y / 10)));
    r += static_cast<char>('0' + static_cast<int>(std::floor(std::fmod(x, 20) / 2)));
    r += static_cast<char>('0' + static_cast<int>(std::floor(std::fmod(y, 10))));
    if (chars >= 6) {
        r += static_cast<char>('a' + static_cast<int>(std::floor(std::fmod(x, 2) * 12)));
        r += static_cast<char>('a' + static_cast<int>(std::floor(std::fmod(y, 1) * 24)));
    }
    return r;
}

void Locator::distanceBearing(double lat1, double lon1, double lat2, double lon2, double& km, double& deg) {
    const double r = kPi / 180;
    const double la1 = lat1 * r, lo1 = lon1 * r, la2 = lat2 * r, lo2 = lon2 * r;
    const double s1 = std::sin((la2 - la1) / 2), s2 = std::sin((lo2 - lo1) / 2);
    const double h = s1 * s1 + std::cos(la1) * std::cos(la2) * s2 * s2;
    km = 2 * 6371 * std::asin(std::min(1.0, std::sqrt(std::max(0.0, h))));
    const double y = std::sin(lo2 - lo1) * std::cos(la2);
    const double x = std::cos(la1) * std::sin(la2) - std::sin(la1) * std::cos(la2) * std::cos(lo2 - lo1);
    deg = std::fmod(std::atan2(y, x) / r + 360, 360.0);
    if (!finite(km)) km = 0;
    if (!finite(deg)) deg = 0;
}

bool Locator::distanceBearing(const std::string& from, const std::string& to, double& km, double& deg) {
    double a, b, c, d;
    if (!toLatLon(from, a, b) || !toLatLon(to, c, d)) return false;
    distanceBearing(a, b, c, d, km, deg);
    return true;
}

int Locator::sector(double deg) {
    if (!finite(deg)) return 0;
    long s = static_cast<long>(std::floor(deg / 45 + 0.5)) % 8;
    if (s < 0) s += 8;
    return static_cast<int>(s);
}

const char* Locator::sectorName(int s) {
    static const char* const kSectors[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return kSectors[((s % 8) + 8) % 8];
}

const City* cities(std::size_t& count) {
    count = sizeof(kCities) / sizeof(kCities[0]);
    return kCities;
}

const City* findCity(const std::string& name) {
    std::size_t n = 0;
    const City* cs = cities(n);
    std::string v = lower(name);
    while (!v.empty() && std::isspace(static_cast<unsigned char>(v[0]))) v.erase(0, 1);
    while (!v.empty() && std::isspace(static_cast<unsigned char>(v[v.size() - 1]))) v.erase(v.size() - 1);
    if (v.empty()) return 0;
    for (std::size_t i = 0; i < n; ++i)
        if (lower(std::string(cs[i].name) + ", " + cs[i].country) == v) return &cs[i];
    for (std::size_t i = 0; i < n; ++i)
        if (lower(cs[i].name) == v) return &cs[i];
    return 0;
}

const City* nearestCity(double lat, double lon, double* kmOut) {
    std::size_t n = 0;
    const City* cs = cities(n);
    const City* best = 0;
    double bd = 1e300;
    if (!finite(lat) || !finite(lon)) return 0;
    for (std::size_t i = 0; i < n; ++i) {
        double km, deg;
        Locator::distanceBearing(lat, lon, cs[i].lat, cs[i].lon, km, deg);
        if (km < bd) { bd = km; best = &cs[i]; }
    }
    if (kmOut && best) *kmOut = bd;
    return best;
}

}  // namespace dec
