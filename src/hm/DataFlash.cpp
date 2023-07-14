#include "glob_var.h"
#include "DataFlash.h"
#include "sci.h"
#include "static_parameters.h"
#include "parameters.h"
#include "mytypedef.h"

unsigned short map_add;

void map_write(void)
{
	short i;

	// DataFlashイレース
	for (i = 0; i < 32; i++)
	{
	}
	// マップデータをDataFlashに書き込む
	for (i = 0; i < 512; i++)
	{
		map_add++;
	}
}

void map_copy(void)
{
	short i;

	// マップデータをRAMにコピー
	for (i = 0; i < 512; i++)
	{
		map_add++;
	}
}

void map_view(void)
{
	signed char i, j;

	SCI_printf("\x1b[0;0H"); // カーソルを0,0に移動
	SCI_printf("\n\r+");
	for (i = 0; i < MAZESIZE_X; i++)
	{
		switch (wall[i][MAZESIZE_Y - 1].north)
		{ // 黒色は"[30m"
		case NOWALL:
			SCI_printf("\x1b[37m  +"); // NOWALL
			break;
		case WALL:
			SCI_printf("\x1b[37m--+"); // WALL
			break;
		case UNKNOWN:
			SCI_printf("\x1b[31m--+"); // UNNOWN
			break;
		default:
			SCI_printf("\x1b[33m--+"); // VWALL
			break;
		}
	}

	SCI_printf("\n\r");
	for (j = (MAZESIZE_Y - 1); j > -1; j--)
	{
		switch (wall[0][j].west)
		{
		case NOWALL:
			SCI_printf("\x1b[37m "); // NOWALL
			break;
		case WALL:
			SCI_printf("\x1b[37m|"); // WALL
			break;
		case UNKNOWN:
			SCI_printf("\x1b[31m|"); // UNNOWN
			break;
		default:
			SCI_printf("\x1b[33m|"); // VWALL
			break;
		}
		for (i = 0; i < MAZESIZE_X; i++)
		{
			switch (wall[i][j].east)
			{
			case NOWALL:
				SCI_printf("\x1b[37m   "); // NOWALL
				break;
			case WALL:
				SCI_printf("\x1b[37m  |"); // WALL
				break;
			case UNKNOWN:
				SCI_printf("\x1b[31m  |"); // UNNOWN
				break;
			default:
				SCI_printf("\x1b[33m  |"); // VWALL
				break;
			}
		}
		SCI_printf("\n\r+");
		for (i = 0; i < MAZESIZE_X; i++)
		{
			switch (wall[i][j].south)
			{
			case NOWALL:
				SCI_printf("\x1b[37m  +"); // NOWALL
				break;
			case WALL:
				SCI_printf("\x1b[37m--+"); // WALL
				break;
			case UNKNOWN:
				SCI_printf("\x1b[31m--+"); // UNNOWN
				break;
			default:
				SCI_printf("\x1b[33m--+"); // VWALL
				break;
			}
		}
		SCI_printf("\n\r");
	}
	return;
}
