#include <kukadu/utils/utils.hpp>
#include <kukadu/learning/projective_simulation/visualization.hpp>

#if VISUALIZATION == 1
    #include <allegro5/allegro_ttf.h>
    #include <allegro5/allegro_font.h>
    #include <allegro5/allegro_primitives.h>
#endif

using namespace std;

namespace kukadu {

    TreeDrawer::TreeDrawer() {

        this->windowXSize = TREEDRAWER_H_WINDOW_X_SIZE;
        this->windowYSize = TREEDRAWER_H_WINDOW_Y_SIZE;

        construct();

    }

    TreeDrawer::TreeDrawer(int windowXSize, int windowYSize) {

        this->windowXSize = windowXSize;
        this->windowYSize = windowYSize;

        construct();

    }

    void TreeDrawer::construct() {

    #if VISUALIZATION == 1

        display = NULL;

        if(!al_init()) {
            cerr << "(TreeDrawer) failed to initialize allegro" << endl;
            throw KukaduException("(TreeDrawer) failed to initialize allegro");
        }

        display = al_create_display(windowXSize, windowYSize);

        al_install_keyboard();
        al_init_font_addon();
        al_init_ttf_addon();

        if(!display) {
            cerr << "(TreeDrawer) failed to create display" << endl;
            throw KukaduException("(TreeDrawer) failed to create display");
        }

        al_clear_to_color(al_map_rgb(0,0,0));
        al_flip_display();

        string textFont = resolvePath("$KUKADU_HOME/src/learning/projective_simulation/fonts/arial.ttf");
        font = al_load_ttf_font(textFont.c_str(), 20, 0);
        if (!font){
            cerr << "(TreeDrawer) Could not load '" + textFont + "'" << endl;
            throw KukaduException(string("(TreeDrawer) Could not load '" + textFont + "'").c_str());
        }

        event_queue = al_create_event_queue();
        if(!event_queue) {
            cerr << "(TreeDrawer) failed to create event_queue" << endl;
            throw KukaduException("(TreeDrawer) failed to create event_queue");
        }

        al_register_event_source(event_queue, al_get_keyboard_event_source());

        textColor.a = 1.0;
        textColor.r = 0;
        textColor.g = 0;
        textColor.b = 0;
        circleColor.r = 240;
        circleColor.g = 240;
        circleColor.b = 255;

    #endif

    }

    TreeDrawer::~TreeDrawer() {
    #if VISUALIZATION == 1
        al_destroy_display(display);
    #endif
    }

    int TreeDrawer::compteXOffset(KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > level) {
    #if VISUALIZATION == 1
        int clipCount = level->size();
        return max(0, (int) ((TREEDRAWER_H_WINDOW_X_SIZE - TREEDRAWER_H_NODE_X_OFFS - TREEDRAWER_H_NODE_X_DIST * clipCount) / 2.0));
    #else
        return 0;
    #endif
    }

    void TreeDrawer::drawTree(KUKADU_SHARED_PTR<ProjectiveSimulator> projSim) {

    #if VISUALIZATION == 1

        int currLayer = 0;

        al_clear_to_color(textColor);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > clipLayers = projSim->getClipLayers();
        int layerCount = clipLayers->size();

        currLayer = 0;

        for(int i = 0; i < clipLayers->size(); ++i) {
            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > level = clipLayers->at(i);

            int currIdx = 0;

            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
            for(it = level->begin(); it != level->end(); ++it) {
                KUKADU_SHARED_PTR<Clip> currClip = *it;

                double totalSubWeight = 0.0;
                int startPosStartNode = compteXOffset(level);
                for(int i = 0; i < currClip->getSubClipCount(); ++i)
                    totalSubWeight += currClip->getWeightByIdx(i);

                int subClipCount = currClip->getSubClipCount();
                for(int i = 0; i < subClipCount; ++i) {

                    KUKADU_SHARED_PTR<Clip> subClip = currClip->getSubClipByIdx(i);
                    int targetLayer = subClip->getLevel();
                    int startPosEndNode = compteXOffset((targetLayer != -1) ? clipLayers->at(targetLayer) : clipLayers->at(clipLayers->size() - 1));

                    auto finalClipLevel = Clip::CLIP_H_LEVEL_FINAL;
                    if(targetLayer == finalClipLevel)
                        targetLayer = layerCount - 1;

                    KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > subLevel = clipLayers->at(targetLayer);
                    int targetIdx = distance(subLevel->begin(), subLevel->find(subClip));

                    al_draw_line(startPosStartNode + TREEDRAWER_H_NODE_X_OFFS + TREEDRAWER_H_NODE_X_DIST * currIdx,
                                 TREEDRAWER_H_NODE_Y_OFFS + TREEDRAWER_H_NODE_Y_DIST * currLayer + TREEDRAWER_H_EDGE_Y_OFFS,
                                 startPosEndNode + TREEDRAWER_H_NODE_X_OFFS + TREEDRAWER_H_NODE_X_DIST * targetIdx,
                                 TREEDRAWER_H_NODE_Y_OFFS + TREEDRAWER_H_NODE_Y_DIST * targetLayer - TREEDRAWER_H_EDGE_Y_OFFS,
                                 circleColor, max(1, (int) (currClip->getWeightByIdx(i) / totalSubWeight * 10.0))
                                 );

                }

                ++currIdx;

            }

            ++currLayer;

        }

        currLayer = 0;
        for(int i = 0; i < clipLayers->size(); ++i) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > level = clipLayers->at(i);

            int currIdx = 0;
            int startPos = compteXOffset(level);
            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
            for(it = level->begin(); it != level->end(); ++it) {
                KUKADU_SHARED_PTR<Clip> currClip = *it;

                drawNode(startPos + TREEDRAWER_H_NODE_X_OFFS + TREEDRAWER_H_NODE_X_DIST * currIdx,
                         TREEDRAWER_H_NODE_Y_OFFS + TREEDRAWER_H_NODE_Y_DIST * currLayer,
                         currClip->toString(), currLayer);

                ++currIdx;

            }

            ++currLayer;

        }

        al_flip_display();
    #endif

    }

    void TreeDrawer::drawNode(int x, int y, std::string text, int level) {

    #if VISUALIZATION == 1

        al_draw_filled_rectangle(x - TREEDRAWER_H_NODE_RADIUS, y - 15, x + TREEDRAWER_H_NODE_RADIUS, y + 15, circleColor);
        al_draw_text(font, textColor, x, y - 10, ALLEGRO_ALIGN_CENTRE, text.c_str());

    #endif

    }

    void TreeDrawer::waitForEnter() {

    #if VISUALIZATION == 1

        while(true) {

            ALLEGRO_EVENT ev;
            al_wait_for_event(event_queue, &ev);

            if(ev.type == ALLEGRO_EVENT_KEY_CHAR) {
                if(ev.keyboard.keycode == ALLEGRO_KEY_ENTER)
                    return;
            }

        }

    #endif

    }

}
