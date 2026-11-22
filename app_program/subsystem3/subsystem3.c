/** @file subsystem3.c
 *
 * @brief Subsystem 3: starts T2 LineBarcode and its 1 ms release.
 *        mbf_nav belongs to its receiver, T6 (subsystem 1).
 */

#include "subsystem3.h"

EXPORT ID tid_line = 0;

_Static_assert(2U == sizeof(nav_cmd_t), "nav_cmd_t is two bytes");
_Static_assert(4U == sizeof(g_line_word), "line word is one word");

EXPORT ER subsystem3_init (void)
{
    return (E_OK);
}

EXPORT ER subsystem3_start (void)
{
    ID id;

    tid_line = app_start_task(line_task, PRI_T2_LINE, APP_CORE, "T2_line");
    if (tid_line < E_OK)
    {
        return (tid_line);
    }
    id = app_start_waker(&tid_line, S3_T2_PERIOD_MS);
    return ((id < E_OK) ? id : E_OK);
}

/*** end of file ***/
