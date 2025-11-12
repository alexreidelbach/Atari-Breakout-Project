
#include "stm32f0xx.h"
#include "ff.h"
#include "lcd.h"
#include "tty.h"
#include "commands.h"
#include <string.h>
#include <stdio.h>

// Declare external function to get the paddle state from main.c.
extern void get_paddle(int *x, int *y, int *width, int *height);
extern void remote(int *p);
extern uint16_t score, h_score;
extern void write_display();

// Define screen dimensions.
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

// typedef struct {
//     uint8_t ch;
//     float freq;
//     uint16_t duration;
// } Note;

// typedef struct {
//     const Note* notes;
//     uint8_t length;
// } Song;

// const Note songA_arr[] = {
//     {2, 0, 10},
//     {1, 262, 250}, 
//     {2, 0, 10},
//     {1, 330, 250}, 
//     {2, 0, 10},
//     {1, 392, 500}, 
//     {2, 0, 10},
//     {1, 523, 250}, 
//     {2, 0, 10},
//     {1, 392, 250}, 
//     {2, 0, 10},
//     {1, 330, 500}, 
//     {2, 0, 10},
//     {1, 349, 250}, 
//     {2, 0, 10},
//     {1, 440, 250}, 
//     {2, 0, 10},
//     {1, 523, 500}, 
//     {2, 0, 10},
// };
// #define SONG_A_LENTH (sizeof(songA_arr)/sizeof(songA_arr[0]))

// const Note songB_arr[] = {
//     {2, 950, 100},
// };
// #define SONG_B_LENGTH (sizeof(songB_arr)/sizeof(songB_arr[0]))

// const Note songC_arr[] = {
//     {1, 600, 100},
//     {2, 800, 100}
// };
// #define SONG_C_LENGTH (sizeof(songC_arr)/sizeof(songC_arr[0]))

// const Note songD_arr[] = {
//     {2, 0, 100},
//     {1, 700, 100},
//     {1, 362, 300},
//     {1, 320, 400},
//     {1, 296, 400},
//     {1, 275, 500},
//     {1, 220, 1000},
//     {1, 0, 100}

// };
// #define SONG_D_LENGTH (sizeof(songD_arr)/sizeof(songD_arr[0]))

// Song background = {songA_arr, SONG_A_LENTH};
// Song breaker = {songB_arr, SONG_B_LENGTH};
// Song paddle = {songC_arr, SONG_C_LENGTH};
// Song failed = {songD_arr, SONG_D_LENGTH};

// volatile Song* currentSong = &songD_arr;
// volatile uint8_t song_index = 0;

// Data structure for the mounted file system.

// #define NA 1000
// #define RATEA 20000
// short int wavetable[N];
// int step0a = 0;
// int offset0a = 0;
// int step1a = 0;
// int offset1a = 0;

// void set_freq(int chan, float f) {
//     if (chan == 1) {
//         if (f == 0.0) {
//             step0a = 0;
//             offset0a = 0;
//         } else
//             step0a = (f * NA / RATEA) * (1<<16);
//     }
//     if (chan == 2) {
//         if (f == 0.0) {
//             step1a = 0;
//             offset1a = 0;
//         } else
//             step1a = (f * NA / RATEA) * (1<<16);
//     }
// }


// void TIM3_IRQHandler(void) {
//     TIM3->SR &= ~TIM_SR_UIF;
//     song_index++;
//     if (song_index >= currentSong->length) {
//         if (currentSong == &background) {
//             song_index = 0;
//             set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
//             TIM3->ARR = currentSong->notes[song_index].duration - 1;
//             TIM3->CNT = 0; 
//         }
//         else {
//         }
//     }
//     else {
//         set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
//         TIM3->ARR = currentSong->notes[song_index].duration - 1;
//         TIM3->CNT = 0; 
//     }
// }


FATFS fs_storage;

typedef union {
    struct {
        unsigned int bisecond:5; // seconds divided by 2
        unsigned int minute:6;
        unsigned int hour:5;
        unsigned int day:5;
        unsigned int month:4;
        unsigned int year:7;
    };
} fattime_t;

// Current time in the FAT file system format.
static fattime_t fattime;

void set_fattime(int year, int month, int day, int hour, int minute, int second)
{
    fattime_t newtime;
    newtime.year = year - 1980;
    newtime.month = month;
    newtime.day = day;
    newtime.hour = hour;
    newtime.minute = minute;
    newtime.bisecond = second/2;
    int len = sizeof newtime;
    memcpy(&fattime, &newtime, len);
}

void advance_fattime(void)
{
    fattime_t newtime = fattime;
    newtime.bisecond += 1;
    if (newtime.bisecond == 30) {
        newtime.bisecond = 0;
        newtime.minute += 1;
    }
    if (newtime.minute == 60) {
        newtime.minute = 0;
        newtime.hour += 1;
    }
    if (newtime.hour == 24) {
        newtime.hour = 0;
        newtime.day += 1;
    }
    if (newtime.month == 2) {
        if (newtime.day >= 29) {
            int year = newtime.year + 1980;
            if ((year % 1000) == 0) { // we have a leap day in 2000
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            } else if ((year % 100) == 0) { // no leap day in 2100
                if (newtime.day > 28)
                newtime.day -= 27;
                newtime.month = 3;
            } else if ((year % 4) == 0) { // leap day for other mod 4 years
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            }
        }
    } else if (newtime.month == 9 || newtime.month == 4 || newtime.month == 6 || newtime.month == 10) {
        if (newtime.day == 31) {
            newtime.day -= 30;
            newtime.month += 1;
        }
    } else {
        if (newtime.day == 0) { // cannot advance to 32
            newtime.day = 1;
            newtime.month += 1;
        }
    }
    if (newtime.month == 13) {
        newtime.month = 1;
        newtime.year += 1;
    }

    fattime = newtime;
}

uint32_t get_fattime(void)
{
    union FattimeUnion {
        fattime_t time;
        uint32_t value;
    };

    union FattimeUnion u;
    u.time = fattime;
    return u.value;
}



void print_error(FRESULT fr, const char *msg)
{
    const char *errs[] = {
            [FR_OK] = "Success",
            [FR_DISK_ERR] = "Hard error in low-level disk I/O layer",
            [FR_INT_ERR] = "Assertion failed",
            [FR_NOT_READY] = "Physical drive cannot work",
            [FR_NO_FILE] = "File not found",
            [FR_NO_PATH] = "Path not found",
            [FR_INVALID_NAME] = "Path name format invalid",
            [FR_DENIED] = "Permision denied",
            [FR_EXIST] = "Prohibited access",
            [FR_INVALID_OBJECT] = "File or directory object invalid",
            [FR_WRITE_PROTECTED] = "Physical drive is write-protected",
            [FR_INVALID_DRIVE] = "Logical drive number is invalid",
            [FR_NOT_ENABLED] = "Volume has no work area",
            [FR_NO_FILESYSTEM] = "Not a valid FAT volume",
            [FR_MKFS_ABORTED] = "f_mkfs aborted",
            [FR_TIMEOUT] = "Unable to obtain grant for object",
            [FR_LOCKED] = "File locked",
            [FR_NOT_ENOUGH_CORE] = "File name is too large",
            [FR_TOO_MANY_OPEN_FILES] = "Too many open files",
            [FR_INVALID_PARAMETER] = "Invalid parameter",
    };
    if (fr < 0 || fr >= sizeof errs / sizeof errs[0])
        printf("%s: Invalid error\n", msg);
    else
        printf("%s: %s\n", msg, errs[fr]);
}

void append(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Specify only one file name to append to.");
        return;
    }
    FIL fil;        /* File object */
    char line[100]; /* Line buffer */
    FRESULT fr;     /* FatFs return code */
    fr = f_open(&fil, argv[1], FA_WRITE|FA_OPEN_EXISTING|FA_OPEN_APPEND);
    if (fr) {
        print_error(fr, argv[1]);
        return;
    }
    printf("To end append, enter a line with a single '.'\n");
    for(;;) {
        fgets(line, sizeof(line)-1, stdin);
        if (line[0] == '.' && line[1] == '\n')
            break;
        int len = strlen(line);
        if (line[len-1] == '\004')
            len -= 1;
        UINT wlen;
        fr = f_write(&fil, (BYTE*)line, len, &wlen);
        if (fr)
            print_error(fr, argv[1]);
    }
    f_close(&fil);
}

void cat(int argc, char *argv[])
{
    for(int i=1; i<argc; i++) {
        FIL fil;        /* File object */
        char line[100]; /* Line buffer */
        FRESULT fr;     /* FatFs return code */

        /* Open a text file */
        fr = f_open(&fil, argv[i], FA_READ);
        if (fr) {
            print_error(fr, argv[i]);
            return;
        }

        /* Read every line and display it */
        while(f_gets(line, sizeof line, &fil))
            printf(line);
        /* Close the file */
        f_close(&fil);
    }
}

void cd(int argc, char *argv[])
{
    if (argc > 2) {
        printf("Too many arguments.");
        return;
    }
    FRESULT res;
    if (argc == 1) {
        res = f_chdir("/");
        if (res)
            print_error(res, "(default path)");
        return;
    }
    res = f_chdir(argv[1]);
    if (res)
        print_error(res, argv[1]);
}

int to_int(char *start, char *end, int base)
{
    int n = 0;
    for( ; start != end; start++)
        n = n * base + (*start - '0');
    return n;
}

static const char *month_name[] = {
        [1] = "Jan",
        [2] = "Feb",
        [3] = "Mar",
        [4] = "Apr",
        [5] = "May",
        [6] = "Jun",
        [7] = "Jul",
        [8] = "Aug",
        [9] = "Sep",
        [10] = "Oct",
        [11] = "Nov",
        [12] = "Dec",
};

void set_fattime(int,int,int,int,int,int);
void date(int argc, char *argv[])
{
    if (argc == 2) {
        char *d = argv[1];
        if (strlen(d) != 14) {
            printf("Date format: YYYYMMDDHHMMSS\n");
            return;
        }
        for(int i=0; i<14; i++)
            if (d[i] < '0' || d[i] > '9') {
                printf("Date format: YYYMMDDHHMMSS\n");
                return;
            }
        int year = to_int(d, &d[4], 10);
        int month = to_int(&d[4], &d[6], 10);
        int day   = to_int(&d[6], &d[8], 10);
        int hour  = to_int(&d[8], &d[10], 10);
        int minute = to_int(&d[10], &d[12], 10);
        int second = to_int(&d[12], &d[14], 10);
        set_fattime(year, month, day, hour, minute, second);
        return;
    }
    int integer = get_fattime();
    union {
        int integer;
        fattime_t ft;
    } u;
    u.integer = integer;
    fattime_t ft = u.ft;
    int year = ft.year + 1980;
    int month = ft.month;
    printf("%d-%s-%02d %02d:%02d:%02d\n",
            year, month_name[month], ft.day, ft.hour, ft.minute, ft.bisecond*2);
}

void dino(int argc, char *argv[])
{
    const char str[] =
    "   .-~~^-.\n"
    " .'  O    \\\n"
    "(_____,    \\\n"
    " `----.     \\\n"
    "       \\     \\\n"
    "        \\     \\\n"
    "         \\     `.             _ _\n"
    "          \\       ~- _ _ - ~       ~ - .\n"
    "           \\                              ~-.\n"
    "            \\                                `.\n"
    "             \\    /               /       \\    \\\n"
    "              `. |         }     |         }    \\\n"
    "                `|        /      |        /       \\\n"
    "                 |       /       |       /          \\\n"
    "                 |      /`- _ _ _|      /.- ~ ^-.     \\\n"
    "                 |     /         |     /          `.    \\\n"
    "                 |     |         |     |             -.   ` . _ _ _ _ _ _\n"
    "                 |_____|         |_____|                ~ . _ _ _ _ _ _ _ >\n";
    puts(str);
}

void input(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Specify only one file name to create.");
        return;
    }
    FIL fil;        /* File object */
    char line[100]; /* Line buffer */
    FRESULT fr;     /* FatFs return code */
    fr = f_open(&fil, argv[1], FA_WRITE|FA_CREATE_NEW);
    if (fr) {
        print_error(fr, argv[1]);
        return;
    }
    printf("To end input, enter a line with a single '.'\n");
    for(;;) {
        fgets(line, sizeof(line)-1, stdin);
        if (line[0] == '.' && line[1] == '\n')
            break;
        int len = strlen(line);
        if (line[len-1] == '\004')
            len -= 1;
        UINT wlen;
        fr = f_write(&fil, (BYTE*)line, len, &wlen);
        if (fr)
            print_error(fr, argv[1]);
    }
    f_close(&fil);
}

void lcd_init(int argc, char *argv[])
{
    LCD_Setup();
}

void ls(int argc, char *argv[])
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;
    const char *path = "";
    int info = 0;
    int i=1;
    do {
        if (argv[i][0] == '-') {
            for(char *c=&argv[i][1]; *c; c++)
                if (*c == 'l')
                    info=1;
            if (i+1 < argc) {
                i += 1;
                continue;
            }
        } else {
            path = argv[i];
        }

        res = f_opendir(&dir, path);                       /* Open the directory */
        if (res != FR_OK) {
            print_error(res, argv[1]);
            return;
        }
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (info) {
                printf("%04d-%s-%02d %02d:%02d:%02d %6ld %c%c%c%c%c ",
                        (fno.fdate >> 9) + 1980,
                        month_name[fno.fdate >> 5 & 15],
                        fno.fdate & 31,
                        fno.ftime >> 11,
                        fno.ftime >> 5 & 63,
                        (fno.ftime & 31) * 2,
                        fno.fsize,
                        (fno.fattrib & AM_DIR) ? 'D' : '-',
                        (fno.fattrib & AM_RDO) ? 'R' : '-',
                        (fno.fattrib & AM_HID) ? 'H' : '-',
                        (fno.fattrib & AM_SYS) ? 'S' : '-',
                        (fno.fattrib & AM_ARC) ? 'A' : '-');
            }
            if (path[0] != '\0')
                printf("%s/%s\n", path, fno.fname);
            else
                printf("%s\n", fno.fname);
        }
        f_closedir(&dir);
        i += 1;
    } while(i<argc);
}

void mkdir(int argc, char *argv[])
{
    for(int i=1; i<argc; i++) {
        FRESULT res = f_mkdir(argv[i]);
        if (res != FR_OK) {
            print_error(res, argv[i]);
            return;
        }
    }
}

void mount(int argc, char *argv[])
{
    FATFS *fs = &fs_storage;
    if (fs->id != 0) {
        print_error(FR_DISK_ERR, "Already mounted.");
        return;
    }
    int res = f_mount(fs, "", 1);
    if (res != FR_OK)
        print_error(res, "Error occurred while mounting");
}

void pwd(int argc, char *argv[])
{
    char line[100];
    FRESULT res = f_getcwd(line, sizeof line);
    if (res != FR_OK)
        print_error(res, "pwd");
    else
        printf("%s\n", line);
}

void rm(int argc, char *argv[])
{
    FRESULT res;
    for(int i=1; i<argc; i++) {
        res = f_unlink(argv[i]);
        if (res != FR_OK)
            print_error(res, argv[i]);
    }
}

void shout(int argc, char *argv[])
{
    char arr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz0123456789\n";
    for(int i=0; i<1000; i++)
        for(int c=0; c<sizeof arr; c++)
            putchar(arr[c]);
}

void clear(int argc, char *argv[])
{
    int value = 0;
    if (argc == 2)
        value = strtoul(argv[1], 0, 16);
    LCD_Clear(value);
}

void drawline(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: line x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawLine(x1,y1,x2,y2,c);
}

void drawrect(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: drawrect x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawRectangle(x1,y1,x2,y2,c);
}

#include <time.h>

u8 getBrightness(u16 color) {
    u8 red   = (color >> 11) & 0x1F; // 5-bit red
    u8 green = (color >> 5)  & 0x3F; // 6-bit green
    u8 blue  = color & 0x1F;         // 5-bit blue

    // Scale components to 0–255.
    u16 r = (red   * 255) / 31;
    u16 g = (green * 255) / 63;
    u16 b = (blue  * 255) / 31;

    return (u8)((r * 299 + g * 587 + b * 114) / 1000);
}

void game(int argc, char *argv[]) {
    LCD_Clear(BLACK);
    // Define dimensions for our letter blocks.
    int base_x = 25;       // intended horizontal starting position (column)
    int base_y = 30;       // intended vertical starting position (row)
    int letterWidth = 30;  // overall width for each letter
    int letterHeight = 75; // overall height for each letter
    int stroke = 5;        // thickness of strokes in pixels
    int gap = 5;           // gap between letters

    int small_base_x = base_x; // Start at the same horizontal position.
    int start_base_y = base_y + letterHeight + 20; // Place START 20 pixels below BREAKOUT.
    int small_letterWidth = 20;   // Narrower letters.
    int small_letterHeight = 50;  // Shorter in height.
    int small_stroke = 3;         // Thinner stroke.
    int small_gap = 3;            // Less gap between letters.


    srand(time(NULL));

    while(1) {
        int pause;
        remote(&pause);
        if(pause == 1) {
            start(NULL, NULL);
        }

        u16 color;
        // Generate random color until it isn't black (0)
        do {
            color = rand() % 0x10000;
            for (volatile int delay = 0; delay < 500000; delay++);
        } while(getBrightness(color) < 32);
        
        // --- Letter T (Segment 1) ---
        int tx = base_x;
        // Top horizontal stroke (full width remains the same)
        LCD_DrawFillRectangle(base_y, tx, base_y + stroke, tx + 30, color);
        // Center vertical stroke.
        // In the original non-mirrored design this was drawn from (tx+13) to (tx+13+stroke).
        // To pre-compensate for the mirror, we shift it by 1 pixel:
        LCD_DrawFillRectangle(base_y, tx + 12, base_y + letterHeight, tx + 17, color);

        // --- Letter U (Segment 2) ---
        int ux = base_x + (letterWidth + gap);
        // For U the vertical strokes swap sides.
        // Pre-mirrored: draw the stroke that should appear on the left on the right.
        LCD_DrawFillRectangle(base_y, ux + 25, base_y + 68, ux + 25 + stroke, color);
        // And vice versa.
        LCD_DrawFillRectangle(base_y, ux, base_y + 68, ux + stroke, color);
        // Bottom horizontal stroke (drawn full width).
        LCD_DrawFillRectangle(base_y + 68, ux, base_y + 70 + stroke, ux + 30, color);

        // --- Letter O (Segment 3) ---
        int ox = base_x + 2 * (letterWidth + gap);
        // Swap vertical strokes as for U.
        LCD_DrawFillRectangle(base_y, ox + 25, base_y + letterHeight, ox + 25 + stroke, color);
        LCD_DrawFillRectangle(base_y, ox, base_y + letterHeight, ox + stroke, color);
        // Top horizontal stroke.
        LCD_DrawFillRectangle(base_y, ox, base_y + stroke, ox + 30, color);
        // Bottom horizontal stroke.
        LCD_DrawFillRectangle(base_y + 70, ox, base_y + 70 + stroke, ox + 30, color);

        // --- Letter K (Segment 4) ---
        int kx = base_x + 3 * (letterWidth + gap);
        // For letter K the final intended stroke is a vertical spine on the left.
        // To achieve that after mirroring we pre-draw the spine on the right.
        LCD_DrawFillRectangle(base_y, kx + 20, base_y + letterHeight, kx + 25, color);
        // Draw the upper diagonal stroke as individual 5x5 cubes.
        // We want to simulate a diagonal that in the original (pre‑mirror) design
        // would have spanned from near the top of the spine to a point a few pixels to its left.
        // In this example, we use 6 cubes.
        int numCubesUpper = 6;
        for (int i = 0; i < numCubesUpper; i++) {
            // Shift left from the spine’s left edge (same as before).
            int cube_x = kx + 20 - i * 3;
            // For the upper diagonal, begin at y = base_y + 35 and step upward by subtracting 7 pixels per cube.
            int cube_y = base_y + 35 - i * 7;
            // Draw the cube.
            LCD_DrawFillRectangle(cube_y, cube_x, cube_y + stroke, cube_x + stroke, color);
        }

        // Draw the lower diagonal stroke as individual 5x5 cubes.
        // This stroke starts from about the midpoint and extends downward.
        // Here we use 6 cubes as well.
        int numCubesLower = 6;
        for (int i = 0; i < numCubesLower; i++) {
            // Again, shift left from the spine’s left edge.
            int cube_x = kx + 20 - i * 3;
            // For the lower diagonal, begin at y = base_y + 35 and step down by 7 pixels per cube.
            int cube_y = base_y + 35 + i * 7;
            // Draw the cube.
            LCD_DrawFillRectangle(cube_y, cube_x, cube_y + stroke, cube_x + stroke, color);
        }


        // --- Letter A (Segment 5) ---
        int ax = base_x + 4 * (letterWidth + gap);
        // Letter A is symmetric. To avoid the mirrored transformation that made it look like an H,
        // we now draw it using the original coordinates.
        LCD_DrawFillRectangle(base_y + 5, ax, base_y + 5 + 70, ax + stroke, color);
        LCD_DrawFillRectangle(base_y + 5, ax + 25, base_y + 5 + 70, ax + 25 + stroke, color);
        LCD_DrawFillRectangle(base_y + 35, ax, base_y + 35 + stroke, ax + 30, color);
        LCD_DrawFillRectangle(base_y, ax, base_y + stroke, ax + 30, color);

        // --- Letter E (Segment 6) ---
        int ex = base_x + 5 * (letterWidth + gap);
        // For letter E the pre-mirror transformation is applied.
        LCD_DrawFillRectangle(base_y, ex + 25, base_y + letterHeight, ex + 30, color);
        LCD_DrawFillRectangle(base_y, ex + 10, base_y + stroke, ex + 30, color);
        LCD_DrawFillRectangle(base_y + 35, ex + 15, base_y + 35 + stroke, ex + 30, color);
        LCD_DrawFillRectangle(base_y + 70, ex + 10, base_y + 70 + stroke, ex + 30, color);

        // --- Letter R (Segment 7) ---
        int rx = base_x + 6 * (letterWidth + gap);
        // Pre-mirror the R strokes.
        LCD_DrawFillRectangle(base_y, rx + 25, base_y + letterHeight, rx + 30, color);
        LCD_DrawFillRectangle(base_y, rx + 10, base_y + stroke, rx + 30, color);
        LCD_DrawFillRectangle(base_y + stroke, rx + 5, base_y + stroke + 25, rx + 10, color);
        LCD_DrawFillRectangle(base_y + 35, rx + 10, base_y + 35 + stroke, rx + 30, color);
        LCD_DrawFillRectangle(base_y + 35, rx + 5, base_y + 35 + 35, rx + 10, color);

        // --- Letter B (Segment 8) ---
        int bx = base_x + 7 * (letterWidth + gap);
        // Pre-mirror for letter B.
        LCD_DrawFillRectangle(base_y, bx + 25, base_y + letterHeight, bx + 30, color);
        LCD_DrawFillRectangle(base_y, bx + 10, base_y + stroke, bx + 30, color);
        LCD_DrawFillRectangle(base_y + stroke, bx + 5, base_y + stroke + 25, bx + 10, color);
        LCD_DrawFillRectangle(base_y + 35, bx + 10, base_y + 35 + stroke, bx + 30, color);
        LCD_DrawFillRectangle(base_y + 70, bx + 10, base_y + 70 + stroke, bx + 30, color);
        LCD_DrawFillRectangle(base_y + 40, bx + 5, base_y + 40 + 25, bx + 10, color);

        // --- Letter S for "START" ---
        // --- Small Letter T (first segment) ---
        color = color + 10;
        int t1x_small = small_base_x + (0 * (small_letterWidth + small_gap));
        // Top horizontal stroke.
        LCD_DrawFillRectangle(start_base_y, t1x_small, start_base_y + small_stroke, t1x_small + small_letterWidth, color);
        // Center vertical stroke.
        LCD_DrawFillRectangle(start_base_y, t1x_small + 8, start_base_y + small_letterHeight, t1x_small + 8 + small_stroke, color);

        // --- Small Letter R (second segment) ---
        int rx_small = small_base_x + (1 * (small_letterWidth + small_gap));
        // Vertical stroke (drawn on the right side).
        LCD_DrawFillRectangle(start_base_y, rx_small + 17, start_base_y + small_letterHeight, rx_small + 20, color);
        // Top horizontal stroke.
        LCD_DrawFillRectangle(start_base_y, rx_small + 7, start_base_y + small_stroke, rx_small + 20, color);
        // Upper diagonal stroke.
        LCD_DrawFillRectangle(start_base_y + small_stroke, rx_small + 3, start_base_y + small_stroke + 17, rx_small + 3 + small_stroke, color);
        // Lower horizontal stroke.
        LCD_DrawFillRectangle(start_base_y + 23, rx_small + 7, start_base_y + 23 + small_stroke, rx_small + 20, color);
        // Lower diagonal stroke.
        LCD_DrawFillRectangle(start_base_y + 23, rx_small + 3, start_base_y + 23 + 25, rx_small + 3 + small_stroke, color);

        // --- Small Letter A (third segment) ---
        int ax_small = small_base_x + (2 * (small_letterWidth + small_gap));
        // Left vertical stroke.
        LCD_DrawFillRectangle(start_base_y + 3, ax_small, start_base_y + 3 + 47, ax_small + small_stroke, color);
        // Right vertical stroke.
        LCD_DrawFillRectangle(start_base_y + 3, ax_small + 17, start_base_y + 3 + 47, ax_small + 17 + small_stroke, color);
        // Middle horizontal stroke.
        LCD_DrawFillRectangle(start_base_y + 23, ax_small, start_base_y + 23 + small_stroke, ax_small + small_letterWidth, color);
        // Top horizontal stroke.
        LCD_DrawFillRectangle(start_base_y, ax_small, start_base_y + small_stroke, ax_small + small_letterWidth, color);

        // --- Small Letter T (fourth segment) ---
        int t2x_small = small_base_x + (3 * (small_letterWidth + small_gap));
        // Top horizontal stroke.
        LCD_DrawFillRectangle(start_base_y, t2x_small, start_base_y + small_stroke, t2x_small + small_letterWidth, color);
        // Center vertical stroke.
        LCD_DrawFillRectangle(start_base_y, t2x_small + 8, start_base_y + small_letterHeight, t2x_small + 8 + small_stroke, color);

        // --- Small Letter S (fifth segment) ---
        int sx_small = small_base_x + (4 * (small_letterWidth + small_gap));
        // Top horizontal stroke.
        LCD_DrawFillRectangle(start_base_y, sx_small, start_base_y + small_stroke, sx_small + small_letterWidth, color);
        // Upper vertical stroke (drawn on the right side).
        LCD_DrawFillRectangle(start_base_y, sx_small + small_letterWidth - small_stroke, start_base_y + small_letterHeight/2, sx_small + small_letterWidth, color);
        // Middle horizontal stroke.
        LCD_DrawFillRectangle(start_base_y + small_letterHeight/2 - small_stroke/2, sx_small, start_base_y + small_letterHeight/2 + small_stroke/2, sx_small + small_letterWidth, color);
        // Lower vertical stroke (drawn on the left side).
        LCD_DrawFillRectangle(start_base_y + small_letterHeight/2, sx_small, start_base_y + small_letterHeight, sx_small + small_stroke, color);
        // Bottom horizontal stroke.
        LCD_DrawFillRectangle(start_base_y + small_letterHeight - small_stroke, sx_small, start_base_y + small_letterHeight, sx_small + small_letterWidth, color);

        // --- Draw the white underline (hover effect) under the small "START" text.
        // Calculate overall width: 5 letters and 4 gaps.
        int underline_x = small_base_x;
        int underline_y = start_base_y + small_letterHeight + 5;  // 5 pixels below the small text
        int underline_width = 5 * small_letterWidth + 4 * small_gap;
        int underline_height = 3;  // Thickness of the underline
        LCD_DrawFillRectangle(underline_y, underline_x, underline_y + underline_height, underline_x + underline_width, WHITE);
    }
}

void title_screen(int argc, char *argv[]) {
}

// Define a structure for the breakout tiles.
typedef struct {
    int x, y;     // Top-left position
    int w, h;     // Width and height
    int active;   // 1 = tile exists, 0 = broken
    u16 color;
} Tile;

#define MAX_TILES 77

int checkEndGame (Tile * tiles, int num_tiles){ //Are all tiles eliminated?
    for (int i = 0; i < num_tiles; i++){
        if (tiles[i].active == 1){
            return 0;
        }
    }
    return 1;
}


void EndScreen(){
    play_fail();
    LCD_Clear(RED);
    // Define dimensions for our letter blocks.
    int base_x = 25;       // intended horizontal starting position (column)
    int base_y = 30;       // intended vertical starting position (row)
    int letterWidth = 30;  // overall width for each letter
    int letterHeight = 75; // overall height for each letter
    int stroke = 5;        // thickness of strokes in pixels
    int gap = 5;           // gap between letters

    int small_base_x = base_x; // Start at the same horizontal position.
    int start_base_y = base_y + letterHeight + 20; // Place START 20 pixels below BREAKOUT.
    int small_letterWidth = 20;   // Narrower letters.
    int small_letterHeight = 50;  // Shorter in height.
    int small_stroke = 3;         // Thinner stroke.
    int small_gap = 3;            // Less gap between letters.
    //INSERT DISPLAY 
    if (score > h_score) {
        h_score = score;
        //Display on 7seg: New High Score: h-score
        write_display();
    }
    write_display();
    score = 0;
    srand(time(NULL));
    //Check for Pause and start game
    while(1){
        int pause;
        remote(&pause);
        if(pause == 1){
            game(NULL,NULL);
        }

        u16 color = WHITE;
        // // Generate random color until it isn't black (0)
        // do {
        //     color = rand() % 0x10000;
        //     for (volatile int delay = 0; delay < 500000; delay++);
        // } while(getBrightness(color) < 32);

        // ----------------------- CENTER “GAME OVER” -----------------------
    // 1) compute total widths
    int bigTextWidth   = 4 * letterWidth     + 3 * gap;
    int smallTextWidth = 4 * small_letterWidth + 3 * small_gap;

    // 2) choose vertical gap between big and small text
    int vertGap = 10;

    // 3) center horizontally
    base_x        = (SCREEN_WIDTH  - bigTextWidth)   / 2;
    small_base_x  = (SCREEN_WIDTH  - smallTextWidth) / 2;

    // 4) center vertically the whole block
    //    total height = letterHeight + vertGap + small_letterHeight
    base_y        = (SCREEN_HEIGHT - (letterHeight + vertGap + small_letterHeight)) / 2;
    start_base_y  = base_y + letterHeight + vertGap;


// ------------ BIG "GAME" ------------

        // --- Letter G (Segment 1) ---
        int gx = base_x + 3 * (letterWidth + gap);
        // Full-height vertical stroke on the right (pre-mirror; appears on left)
        LCD_DrawFillRectangle(base_y,    gx + 25,     base_y + letterHeight, gx + 25 + stroke, color);
        // Top horizontal stroke (full width)
        LCD_DrawFillRectangle(base_y,    gx,          base_y + stroke,       gx + 30,          color);
        // Bottom-half vertical stroke on the left (pre-mirror; appears on right)
        LCD_DrawFillRectangle(base_y + 35, gx,         base_y + letterHeight, gx + stroke,      color);
        // Middle horizontal stroke (left half only)
        LCD_DrawFillRectangle(base_y + 35, gx + stroke, base_y + 35 + stroke,  gx + 15,          color);
        // Bottom horizontal stroke (full width)
        LCD_DrawFillRectangle(base_y + 70, gx,          base_y + 70 + stroke,  gx + 30,          color);

        // --- Letter A (Segment 2) ---
        int ax = base_x + 2 * (letterWidth + gap);
        LCD_DrawFillRectangle(base_y + 5,  ax,             base_y + letterHeight, ax + stroke,      color);
        LCD_DrawFillRectangle(base_y + 5,  ax + 25,        base_y + letterHeight, ax + 25 + stroke, color);
        LCD_DrawFillRectangle(base_y + 35, ax,             base_y + 35 + stroke,       ax + 30,          color);
        LCD_DrawFillRectangle(base_y,      ax,             base_y + stroke,            ax + 30,          color);

        // --- Letter M (Segment 3) ---
        int mx = base_x + 1 * (letterWidth + gap);
        LCD_DrawFillRectangle(base_y,    mx,      base_y + letterHeight, mx + stroke,       color);
        LCD_DrawFillRectangle(base_y,    mx + 25, base_y + letterHeight, mx + 25 + stroke,  color);
        for (int i = 0; i < 6; i++) {
            int cube_x_L = mx + i * 3;
            int cube_y_L = base_y + i * 7;
            LCD_DrawFillRectangle(cube_y_L, cube_x_L, cube_y_L + stroke, cube_x_L + stroke, color);
            int cube_x_R = mx + 25 - i * 3;
            int cube_y_R = base_y + i * 7;
            LCD_DrawFillRectangle(cube_y_R, cube_x_R, cube_y_R + stroke, cube_x_R + stroke, color);
        }

        // --- Letter E (Segment 4) ---
        int ex = base_x;
        LCD_DrawFillRectangle(base_y,    ex + 25,     base_y + letterHeight, ex + 30, color);
        LCD_DrawFillRectangle(base_y,    ex + 10,     base_y + stroke,       ex + 30, color);
        LCD_DrawFillRectangle(base_y + 35, ex + 15,     base_y + 35 + stroke,  ex + 30, color);
        LCD_DrawFillRectangle(base_y + 70, ex + 10,     base_y + 70 + stroke,  ex + 30, color);


// ------------ SMALL "OVER" ------------

        //color += 10;

        // --- Small Letter O (Segment 5) ---
        int ox = small_base_x + 3 * (small_letterWidth + small_gap);
        LCD_DrawFillRectangle(start_base_y,                       ox,                                        start_base_y + small_letterHeight, ox + small_stroke,            color);
        LCD_DrawFillRectangle(start_base_y,                       ox + small_letterWidth - small_stroke,     start_base_y + small_letterHeight, ox + small_letterWidth,       color);
        LCD_DrawFillRectangle(start_base_y,                       ox,                                        start_base_y + small_stroke,        ox + small_letterWidth,       color);
        LCD_DrawFillRectangle(start_base_y + small_letterHeight - small_stroke, ox,                      start_base_y + small_letterHeight, ox + small_letterWidth,       color);

        // --- Small Letter V (Segment 6) ---
        int vx = small_base_x + 2 * (small_letterWidth + small_gap);
        for (int i = 0; i < 5; i++) {
            int cxs_L = vx + i * 3;
            int cys_L = start_base_y + i * 7;
            LCD_DrawFillRectangle(cys_L, cxs_L, cys_L + small_stroke, cxs_L + small_stroke, color);
            int cxs_R = vx + small_letterWidth - small_stroke - i * 3;
            int cys_R = start_base_y + i * 7;
            LCD_DrawFillRectangle(cys_R, cxs_R, cys_R + small_stroke, cxs_R + small_stroke, color);
        }

        // --- Small Letter E (Segment 7) ---
        int ex2 = small_base_x + 1 * (small_letterWidth + small_gap);
        LCD_DrawFillRectangle(start_base_y,                          ex2 + small_letterWidth - small_stroke, start_base_y + small_letterHeight, ex2 + small_letterWidth, color);
        LCD_DrawFillRectangle(start_base_y,                          ex2,                                 start_base_y + small_stroke,        ex2 + small_letterWidth, color);
        LCD_DrawFillRectangle(start_base_y + small_letterHeight/2 - small_stroke/2, ex2, start_base_y + small_letterHeight/2 + small_stroke/2, ex2 + small_letterWidth, color);
        LCD_DrawFillRectangle(start_base_y + small_letterHeight - small_stroke,       ex2, start_base_y + small_letterHeight, ex2 + small_letterWidth, color);

        // --- Small Letter R (Segment 8) ---
        int rx = small_base_x;
        LCD_DrawFillRectangle(start_base_y,
                             rx + 17,
                             start_base_y + small_letterHeight,
                             rx + 20,
                             color);
        LCD_DrawFillRectangle(start_base_y,
                             rx + 7,
                             start_base_y + small_stroke,
                             rx + 20,
                             color);
        LCD_DrawFillRectangle(start_base_y + small_stroke,
                             rx + 3,
                             start_base_y + small_stroke + 17,
                             rx + 3 + small_stroke,
                             color);
        LCD_DrawFillRectangle(start_base_y + 23,
                             rx + 7,
                             start_base_y + 23 + small_stroke,
                             rx + 20,
                             color);
        LCD_DrawFillRectangle(start_base_y + 23,
                             rx + 3,
                             start_base_y + 23 + 25,
                             rx + 3 + small_stroke,
                             color);

        
        }
}

void start(int argc, char *argv[]) {
    Tile tiles[MAX_TILES];
    int num_tiles = 0;

    LCD_Clear(BLACK);
    
    //Starts background song for game
    play_background();

    // Create the grid of tiles.
    // (Using your original loop—but here we interpret j as the x-coordinate and i as the y-coordinate.)
    // This code makes the array of breakout tiles
    srand(time(NULL));
    for(int i = 0; i <= 200; i += 30) {  // This controls the y axis, Column
        for(int j = 0; j <= 300; j += 30) {  // This controls the x axis, Row
            u16 randomColor;
            // Generate random color until it isn't black (0)
            do {
                randomColor = rand() % 0x10000;
            } while(getBrightness(randomColor) < 32);
            LCD_DrawFillRectangle(i, j, i + 5, j + 20, randomColor);
            // Save the tile info.
            tiles[num_tiles].x = i;
            tiles[num_tiles].y = j;
            tiles[num_tiles].w = 20;
            tiles[num_tiles].h = 5;
            tiles[num_tiles].active = 1;
            tiles[num_tiles].color = randomColor;
            num_tiles++;
        }
    }
    
    // Initialize ball parameters.
    int ball_x = 200, ball_y = 160;  // X moves up and down, Y moves right and Left
    int ball_w = 5, ball_h = 5;
    //int ball_dx = 1, ball_dy = 1;
    int ball_dx = -1, ball_dy = 0;
    
    // Variables to track previous positions so we can erase them.
    int prev_ball_x = -1, prev_ball_y = -1;
    int prev_paddle_x = -1, prev_paddle_y = -1, prev_paddle_w = 0, prev_paddle_h = 0;

    int lifecounter = 3;
    write_display();
    
    while (1) {
        // --- Update ball position ---
        write_display();
        // ball_x is vertical, ball_y is horizontal.
        int pause;
        remote(&pause);
        if(pause == 1) {
            ball_x = ball_x;
            ball_y = ball_y;
        } else {
            ball_x += ball_dx;  // vertical movement
            ball_y += ball_dy;  // horizontal movement
        }
    
        // --- Wall collisions ---
        // Top/bottom walls (vertical boundaries): use ball_x and ball_h with SCREEN_HEIGHT.
        if (ball_x <= 0) {
            ball_x = 0;
            ball_dx = -ball_dx;
        } else if (ball_x + ball_h >= SCREEN_HEIGHT) {
            ball_x = SCREEN_HEIGHT - ball_h;
            ball_dx = -ball_dx;
            printf("   / \\__\n"
                "  (    @\\___\n"
                "  /     '   O\n"
                " /   (_____/\n"
                "/_____/   U\n"
                "--------------\n"
                "You hit the bottom\n"
                "--- GAME RESET ---\n");
            // store current score into final score
            // reset current score 
            
            // todo: add game over or reset to title screen and display h_score on ssd
            lifecounter--;
            if (lifecounter == 0){
                write_display();
                EndScreen(score);
            }
        }
    
        // Left/right walls (horizontal boundaries): use ball_y and ball_w with SCREEN_WIDTH.
        if (ball_y <= 0) {
            ball_y = 0;
            ball_dy = -ball_dy;
        } else if (ball_y + ball_w >= SCREEN_WIDTH) {
            ball_y = SCREEN_WIDTH - ball_w;
            ball_dy = -ball_dy;
        }
    
        // --- Get current paddle state (updated by keypad ISR) ---
        // Assume paddle_x is vertical and paddle_y is horizontal.
        // Also, paddle_h is the vertical size and paddle_w is the horizontal size.
        int paddle_x, paddle_y, paddle_w, paddle_h;
        get_paddle(&paddle_x, &paddle_y, &paddle_w, &paddle_h);
    
        // --- Collision detection: Ball vs. Paddle ---
        if (ball_x < paddle_x + paddle_w && ball_x + ball_w > paddle_x &&
            ball_y < paddle_y + paddle_h && ball_y + ball_h > paddle_y) {
            // Reverse horizontal velocity (as in your working paddle bounce).
            ball_dx = -ball_dx;
            // Adjust vertical velocity using scaled paddle movement delta.
            ball_dy = (prev_paddle_y - paddle_y) / 2;
            // Audio for paddle collision
            play_paddle();
        }
    
        // --- Collision detection: Ball vs. Tiles ---
        // Tiles: tile.x is vertical, tile.y is horizontal.
        // tile.h is vertical size and tile.w is horizontal size.
        for (int i = 0; i < num_tiles; i++) {
            if (tiles[i].active) {
                if (ball_x < tiles[i].x + tiles[i].h && ball_x + ball_h > tiles[i].x &&
                    ball_y < tiles[i].y + tiles[i].w && ball_y + ball_w > tiles[i].y) {
    
                    // Compute overlaps on each axis.
                    int verticalOverlapTop    = (ball_x + ball_h) - tiles[i].x;
                    int verticalOverlapBottom = (tiles[i].x + tiles[i].h) - ball_x;
                    int horizontalOverlapLeft = (ball_y + ball_w) - tiles[i].y;
                    int horizontalOverlapRight= (tiles[i].y + tiles[i].w) - ball_y;
    
                    int minVertical   = (verticalOverlapTop < verticalOverlapBottom) ? verticalOverlapTop : verticalOverlapBottom;
                    int minHorizontal = (horizontalOverlapLeft < horizontalOverlapRight) ? horizontalOverlapLeft : horizontalOverlapRight;
    
                    // Reverse the velocity corresponding to the smallest overlap.
                    if (minHorizontal < minVertical) {
                        // Horizontal collision: adjust horizontal position and reverse ball_dy.
                        if (ball_dy > 0)
                            ball_y = tiles[i].y - ball_w;
                        else
                            ball_y = tiles[i].y + tiles[i].w;
                        ball_dy = -ball_dy;
                    } else {
                        // Vertical collision: adjust vertical position and reverse ball_dx.
                        if (ball_dx > 0)
                            ball_x = tiles[i].x - ball_h;
                        else
                            ball_x = tiles[i].x + tiles[i].h;
                        ball_dx = -ball_dx;
                    }
    
                    // Mark the entire tile as broken and erase it.
                    tiles[i].active = 0;
                    //
                    // The tile was originally drawn as:
                    // LCD_DrawFillRectangle(i, j, i+5, j+20, randomColor);
                    // Here, tile.x is the vertical coordinate and tile.y is horizontal.
                    LCD_DrawFillRectangle(tiles[i].x, tiles[i].y, tiles[i].x + 5, tiles[i].y + 20, 0x0000);
                    printf("   / \\__\n"
                        "  (    @\\___\n"
                        "  /         O\n"
                        " /   (_____/\n"
                        "/_____/   U\n"
                        "--------------\n"
                        "You hit a Tile\n");
                    //
                    score += 5;
                    write_display();
                    if (checkEndGame(tiles, num_tiles)){
                        start(NULL,NULL);
                        return;
                    }
                    // Audio for brick collision
                    play_breaker();
                    break;  // Process one tile collision per update.
                }
            }
        }
    
        // --- Redraw scene ---
        // Erase previous ball and paddle.
        if (prev_ball_x != -1) {
            LCD_DrawFillRectangle(prev_ball_x, prev_ball_y,
                                  prev_ball_x + ball_h, prev_ball_y + ball_w, 0x0000);
        }
        if (prev_paddle_x != -1) {
            LCD_DrawFillRectangle(prev_paddle_x, prev_paddle_y, prev_paddle_x + prev_paddle_w, prev_paddle_y + prev_paddle_h, 0x0000);
        }
    
        // Draw the paddle.
        LCD_DrawFillRectangle(paddle_x, paddle_y, paddle_x + paddle_w, paddle_y + paddle_h, 0xFFFF);
    
        // Draw the ball (using red, 0xF800).
        LCD_DrawFillRectangle(ball_x, ball_y,
                              ball_x + ball_h, ball_y + ball_w, 0xF800);
    
        // Save current positions for the next iteration's erase.
        prev_ball_x = ball_x;
        prev_ball_y = ball_y;
        prev_paddle_x = paddle_x;
        prev_paddle_y = paddle_y;
        prev_paddle_h = paddle_h;
        prev_paddle_w = paddle_w;
    
        // Debug prints (optional).
        //printf("Ball: x=%d, y=%d, dx=%d, dy=%d\r\n", ball_x, ball_y, ball_dx, ball_dy);
        //printf("Paddle: x=%d, y=%d\r\n", paddle_x, paddle_y);
    
        // Delay to control refresh rate.
        for (volatile int delay = 0; delay < 50000; delay++);
    }
}

void drawfillrect(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: drawfillrect x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawFillRectangle(x1,y1,x2,y2,c);
}

void add(int argc, char *argv[])
{
  int sum = 0;
  for(int i=1; i < argc; i++) {
      sum += strtol(argv[i], 0, 0);
  }
  printf("The sum is %d\n", sum);
}

void mul(int argc, char *argv[])
{
  int prod = 1;
  for(int i=1; i < argc; i++) {
    prod *= strtol(argv[i], 0, 0);
  }
  printf("The product is %d\n", prod);
}

struct commands_t cmds[] = {
        { "append", append },
        { "cat", cat },
        { "cd", cd },
        { "date", date },
        { "dino", dino },
        { "input", input },
        { "lcd_init", lcd_init },
        { "ls", ls },
        { "mkdir", mkdir },
        { "mount", mount },
        { "pwd", pwd },
        { "rm", rm },
        { "shout", shout },
        { "clear",    clear },
        { "drawline", drawline },
        { "drawrect", drawrect },
        {"start", start},
        {"game", game},
        {"title", title_screen},
        { "drawfillrect", drawfillrect },
};

// A weak definition that can be overridden by a better one.
// Replace this with your own usercmds entirely.
__attribute((weak)) struct commands_t usercmds[] = {
        { 0, 0 }
};

void exec(int argc, char *argv[])
{
    //for(int i=0; i<argc; i++)
    //    printf("%d: %s\n", i, argv[i]);
    for(int i=0; usercmds[i].cmd != 0; i++)
        if (strcmp(usercmds[i].cmd, argv[0]) == 0) {
            usercmds[i].fn(argc, argv);
            return;
        }
    for(int i=0; i<sizeof cmds/sizeof cmds[0]; i++)
        if (strcmp(cmds[i].cmd, argv[0]) == 0) {
            cmds[i].fn(argc, argv);
            return;
        }
    printf("%s: No such command.\n", argv[0]);
}

void parse_command(char *c)
{
    char *argv[20];
    int argc=0;
    int skipspace=1;
    for(; *c; c++) {
        if (skipspace) {
            if (*c != ' ' && *c != '\t') {
                argv[argc++] = c;
                skipspace = 0;
            }
        } else {
            if (*c == ' ' || *c == '\t') {
                *c = '\0';
                skipspace=1;
            }
        }
    }
    if (argc > 0) {
        argv[argc] = "";
        exec(argc, argv);
    }
}


static void insert_echo_string(const char *s)
{
    // This puts a string into the *input* stream, so it can be edited.
    while(*s)
        insert_echo_char(*s++);
}

void command_shell(void)
{
    printf("\nEnter current "); fflush(stdout);
    insert_echo_string("date 20240213000000");
    
    char line[100];
    fgets(line, 99, stdin);
    line[99] = '\0';
    int len = strlen(line);
    if (line[len-1] == '\n')
        line[len-1] = '\0';
    parse_command(line);

    puts("This is the STM32 command shell.");
    puts("Type 'mount' before trying any file system commands.");
    puts("Type 'lcd_init' before trying any draw commands.");
    for(;;) {
        printf("> ");
        fgets(line, 99, stdin);
        line[99] = '\0';
        len = strlen(line);
        if (line[len-1] == '\n')
            line[len-1] = '\0';
        parse_command(line);
    }
}
