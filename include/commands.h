#ifndef __COMMANDS_H__
#define __COMMANDS_H__

struct commands_t {
    const char *cmd;
    void      (*fn)(int argc, char *argv[]);
};

void command_shell(void);
void lcd_init(int argc, char *argv[]);
void game(int argc, char *argv[]);
void play_background(void);
void play_breaker(void);
void play_paddle(void);
void play_fail(void);
//void TIM3_IRQHandler(void);
// Put lcd_init here

#endif /* __COMMANDS_H_ */
