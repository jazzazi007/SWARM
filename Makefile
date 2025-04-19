CC = cc
CFALGS = -Wall -Wextra -Wno-address-of-packed-member
# adapt the following falgs...
# gcc -o gnc gnc.c -lSDL2 -lm
SRCS = src/udp_connection.c src/rc_override.c src/readautopilot.c src/send_autopilot.c \
		src/display.c src/swarm.c
OBJS = $(SRCS:.c=.o)
NAME = swarm
all: $(NAME)
$(NAME): $(OBJS)
	$(CC) $(CFALGS) $(OBJS) -o $(NAME) -lSDL2 -lSDL2_ttf -lm
%.o: %.c
	$(CC) $(CFALGS) -c $< -o $@
clean:
	rm -f $(OBJS)
fclean: clean
	rm -f $(NAME)
re: fclean all
.PHONY: all clean fclean re