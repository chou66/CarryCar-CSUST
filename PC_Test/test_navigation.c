#include <stdio.h>
#include <stdint.h>
#include "navigation.h"

static int pass_n, fail_n;
static void check(int ok, const char *s)
{ printf("[%s] %s\n", ok ? "PASS":"FAIL", s); ok ? pass_n++ : fail_n++; }

static uint8_t run(Navigation_t *n)
{
    uint8_t a,b;
    while (!Navigation_IsDone(n)) {
        if (!Navigation_GetCurrentEdge(n,&a,&b)) return 0U;
        printf("Edge : N%u -> N%u\n",a,b);
        if (!Navigation_EdgeReached(n)) return 0U;
    }
    return 1U;
}

int main(void)
{
    Navigation_t n;
    const PathResult_t *p;
    uint8_t a,b,i,ok;
    static const uint8_t targets[]={8,7,2,6,8,2,6,3};

    printf("\n=== Navigation module PC test ===\n");

    check(Navigation_Init(&n,9,0),"Init N9");
    check(Navigation_GetState(&n)==NAV_STATE_IDLE,"IDLE");
    check(Navigation_PlanTo(&n,3),"Plan N9->N3");
    check(Navigation_GetCurrentEdge(&n,&a,&b)&&a==9&&b==6,"First edge 9->6");
    check(run(&n),"Execute path");
    check(Navigation_GetCurrentNode(&n)==3&&Navigation_IsDone(&n),"Arrive N3");

    check(Navigation_Init(&n,5,0)&&Navigation_PlanTo(&n,5),"Same-node plan");
    check(Navigation_IsDone(&n)&&Navigation_GetCurrentNode(&n)==5,"Same-node DONE");

    /* 固定动线偏好：扫码 N6->原料 N8 优先经 N9，粗加工 N2->暂存 N4 优先经 N1 */
    Navigation_Init(&n,6,0);
    check(Navigation_PlanTo(&n,8),"Plan N6->N8 via N9");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[0]==6&&p->node[1]==9&&p->node[2]==8,
          "Preferred route N6->N9->N8");
    check(run(&n)&&Navigation_GetCurrentNode(&n)==8,"Preferred route arrives N8");

    Navigation_Init(&n,2,0);
    check(Navigation_PlanTo(&n,4),"Plan N2->N4 via N1");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[0]==2&&p->node[1]==1&&p->node[2]==4,
          "Preferred route N2->N1->N4");
    check(run(&n)&&Navigation_GetCurrentNode(&n)==4,"Preferred route arrives N4");

    /* 偏好路线封路时回退普通最短路：节点封路 (bit8=N9, bit0=N1) 与边封路 (bit20=6-9, bit15=1-4) */
    Navigation_Init(&n,6,(1UL<<8));
    check(Navigation_PlanTo(&n,8),"Plan N6->N8 with N9 blocked");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[1]==5&&p->node[2]==8,
          "Fallback N6->N5->N8 when N9 blocked");

    Navigation_Init(&n,2,(1UL<<0));
    check(Navigation_PlanTo(&n,4),"Plan N2->N4 with N1 blocked");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[1]==5&&p->node[2]==4,
          "Fallback N2->N5->N4 when N1 blocked");

    Navigation_Init(&n,6,(1UL<<20));
    check(Navigation_PlanTo(&n,8),"Plan N6->N8 with edge 6-9 blocked");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[1]==5&&p->node[2]==8,
          "Fallback N6->N5->N8 when edge 6-9 blocked");

    Navigation_Init(&n,2,(1UL<<15));
    check(Navigation_PlanTo(&n,4),"Plan N2->N4 with edge 1-4 blocked");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==3&&p->node[1]==5&&p->node[2]==4,
          "Fallback N2->N5->N4 when edge 1-4 blocked");

    Navigation_Init(&n,9,0); Navigation_PlanTo(&n,3);
    check(Navigation_Pause(&n)&&Navigation_GetState(&n)==NAV_STATE_PAUSED,"Pause");
    check(!Navigation_GetCurrentEdge(&n,&a,&b)&&!Navigation_EdgeReached(&n),
          "Paused rejects progress");
    check(Navigation_Resume(&n)&&run(&n),"Resume and finish");

    Navigation_Init(&n,9,(1UL<<20));
    check(Navigation_PlanTo(&n,3),"Plan with z21 blocked");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==5&&p->node[0]==9&&p->node[1]==8&&
          p->node[2]==5&&p->node[3]==2&&p->node[4]==3,
          "Reroute 9->8->5->2->3");
    check(run(&n)&&Navigation_GetCurrentNode(&n)==3,"Reroute arrives N3");

    Navigation_Init(&n,9,0x00104000UL);
    check(!Navigation_PlanTo(&n,3),"NO PATH rejected");
    check(Navigation_GetState(&n)==NAV_STATE_NO_PATH&&Navigation_HasError(&n),
          "NO_PATH state");
    check(Navigation_GetCurrentNode(&n)==9,"NO_PATH keeps N9");

    Navigation_Init(&n,9,0); Navigation_PlanTo(&n,3);
    check(Navigation_SetObstacleMask(&n,(1UL<<20))&&Navigation_Replan(&n),
          "Dynamic replan after map update");
    p=Navigation_GetPath(&n);
    check(p&&p->valid&&p->length==5&&p->node[1]==8&&p->node[4]==3,
          "Replan changed route");
    check(run(&n)&&Navigation_GetCurrentNode(&n)==3,"Replan arrives N3");

    Navigation_Init(&n,9,0); ok=1U;
    for(i=0;i<(uint8_t)(sizeof(targets)/sizeof(targets[0]));i++) {
        printf("Stage %u: N%u -> N%u\n",(unsigned)(i+1),
               Navigation_GetCurrentNode(&n),targets[i]);
        if(!Navigation_PlanTo(&n,targets[i])||!run(&n)||
           Navigation_GetCurrentNode(&n)!=targets[i]) { ok=0U; break; }
    }
    check(ok&&Navigation_GetCurrentNode(&n)==3,"Multi-stage mission skeleton");

    Navigation_Init(&n,9,0); Navigation_PlanTo(&n,3); Navigation_Abort(&n);
    check(Navigation_GetState(&n)==NAV_STATE_IDLE&&
          Navigation_GetCurrentNode(&n)==9&&Navigation_GetTargetNode(&n)==0,
          "Abort preserves confirmed node");
    check(Navigation_SetCurrentNode(&n,5)&&Navigation_PlanTo(&n,3)&&run(&n)&&
          Navigation_GetCurrentNode(&n)==3,"Position correction then navigate");

    printf("\nPASS: %d\nFAIL: %d\nTOTAL: %d\n",pass_n,fail_n,pass_n+fail_n);
    if(!fail_n){ puts("ALL NAVIGATION MODULE TESTS PASSED"); return 0; }
    puts("NAVIGATION MODULE TEST FAILED"); return 1;
}
