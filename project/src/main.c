/*
memory allocation, global:

const, initialized      => .rodata
const, not initialized  => .rodata
var, initialized        => .data
var, not initialized    => .bss
*/
const int gci = 5;              /* .rodata */
const int gcu;                  /* .rodata */
int gvi = 10;                   /* .data */
int gvu;                        /* .bss */

int main(void)
{
    /*
    memory allocation, static:

    static const, initialized      => .rodata
    static const, not initialized  => .rodata
    static var, initialized        => .data
    statuc var, not initialized    => .bss
    */
    static const int sci = 2;   /* .rodata */
    static const int sc;        /* .rodata */
    static int svi = 3;         /* .data */
    static int sv;              /* .bss */

    int v = 100;
    
    gvu = 2;
    
    while(svi < 10000)
    {
        svi += (gci + gvu + gvi + sci + sc + sv);
        sv += 1;
    }
    
    while(1)
    {
        asm ("nop");
    }
    
    
    return 0;
}

void SysTick_Handler(void)
{
    while(1)
    {
        asm ("nop");
    }
}
