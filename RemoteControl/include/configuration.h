//////////////
// Settings //
//////////////
#define USE_MAIN_TAB_VIEW
#ifdef USE_MAIN_TAB_VIEW
    //#define SHOW_TABS_AT_LEFT
    //#define USE_HOME_PAGE
    #define USE_MODULE_SETTINGS
#endif

/////////////
// Modules //
/////////////
//#define SHOW_TOP_BAR
#define USE_MODULE_CONTROLS

//////////////////
// Running mode //
//////////////////

// Debug
//#define MODE_DEBUG_FULL
#define MODE_DEBUG
//#define MODE_RELEASE_INFO
//#define MODE_RELEASE

// Multithread
#define USE_MULTI_THREAD
#define DISPLAY_AT_CORE1

// Language
//#define LANG_EN
#define LANG_GR

////////////
// EEPROM //
////////////
// Enable EEPROM to save settings in EEPROM
#define ENABLE_EEPROM
// In case you want to clear the EEPROM, enable CLEAR_SETTINGS and run one.
// Then comment out and run again
// #define CLEAR_SETTINGS
