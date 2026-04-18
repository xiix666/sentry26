#include <behaviortree_cpp/bt_factory.h>

// 包含所有行为节点头文件
#include "../behaviors/GoToArea1.hpp"
#include "../behaviors/GoToArea2.hpp"
#include "../behaviors/GoToArea2yellow.hpp"
#include "../behaviors/GoToArea3.hpp"
#include "../behaviors/GoToArea4.hpp"
#include "../behaviors/GoToArea4red.hpp"
#include "../behaviors/GoToArea4yellow.hpp"
#include "../behaviors/StayAtSupplyPoint.hpp"
#include "../behaviors/Area1Patrol3.hpp"
#include "../conditions/Area2YellowTimer.hpp"
#include "../behaviors/Area3Patrol3.hpp"
#include "../behaviors/FollowHero.hpp"
#include "../behaviors/EvadeEnemyInfantry.hpp"
#include "../behaviors/CostmapCheckGoal.hpp"
#include "../behaviors/TestQif.hpp"
// 行为节点库的初始化函数（BT 4.x 插件加载时查找的符号名）
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    // 注册所有行为节点 
    //factory.registerNodeType<rm2026_decision::GoArea1>("GoArea1");
    factory.registerNodeType<rm2026_decision::GoToArea1>("GoToArea1");
    factory.registerNodeType<rm2026_decision::GoToArea2>("GoToArea2");
    factory.registerNodeType<rm2026_decision::GoToArea2yellow>("GoToArea2yellow");
    factory.registerNodeType<rm2026_decision::GoToArea3>("GoToArea3");
    factory.registerNodeType<rm2026_decision::GoToArea4>("GoToArea4");
    factory.registerNodeType<rm2026_decision::GoToArea4red>("GoToArea4red");
    factory.registerNodeType<rm2026_decision::GoToArea4yellow>("GoToArea4yellow");
    factory.registerNodeType<rm2026_decision::StayAtSupplyPoint>("StayAtSupplyPoint");
    factory.registerNodeType<rm2026_decision::Area1Patrol3>("Area1Patrol3");
    factory.registerNodeType<rm2026_decision::Area2YellowTimer>("Area2YellowTimer");
    factory.registerNodeType<rm2026_decision::Area3Patrol3>("Area3Patrol3");
    factory.registerNodeType<rm2026_decision::FollowHero>("FollowHero");
    factory.registerNodeType<rm2026_decision::EvadeEnemyInfantry>("EvadeEnemyInfantry");
    factory.registerNodeType<rm2026_decision::CostmapCheckGoal>("CostmapCheckGoal");
    factory.registerNodeType<rm2026_decision::TestQif>("TestQif");
}