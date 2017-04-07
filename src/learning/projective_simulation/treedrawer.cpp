#include <kukadu/utils/utils.hpp>
#include <kukadu/learning/projective_simulation/visualization.hpp>

#include <lemon/math.h>
#include <lemon/list_graph.h>
#include <lemon/graph_to_eps.h>

#if VISUALIZATION == 1
    #include <allegro5/allegro_ttf.h>
    #include <allegro5/allegro_font.h>
    #include <allegro5/allegro_primitives.h>
#endif

using namespace std;
using namespace lemon;

namespace kukadu {

    GraphDrawer::GraphDrawer() {

        Palette palette;
        Palette paletteW(true);

        // Create a small digraph
        ListDigraph g;
        typedef ListDigraph::Node Node;
        typedef ListDigraph::NodeIt NodeIt;
        typedef ListDigraph::Arc Arc;
        typedef dim2::Point<int> Point;

        Node n1=g.addNode();
        Node n2=g.addNode();
        Node n3=g.addNode();
        Node n4=g.addNode();
        Node n5=g.addNode();

        ListDigraph::NodeMap<Point> coords(g);
        ListDigraph::NodeMap<double> sizes(g);
        ListDigraph::NodeMap<int> colors(g);
        ListDigraph::NodeMap<int> shapes(g);
        ListDigraph::ArcMap<int> acolors(g);
        ListDigraph::ArcMap<int> widths(g);

        coords[n1]=Point(50,50);  sizes[n1]=1; colors[n1]=1; shapes[n1]=0;
        coords[n2]=Point(50,70);  sizes[n2]=2; colors[n2]=2; shapes[n2]=2;
        coords[n3]=Point(70,70);  sizes[n3]=1; colors[n3]=3; shapes[n3]=0;
        coords[n4]=Point(70,50);  sizes[n4]=2; colors[n4]=4; shapes[n4]=1;
        coords[n5]=Point(85,60);  sizes[n5]=3; colors[n5]=5; shapes[n5]=2;

        Arc a;

        a=g.addArc(n1,n2); acolors[a]=0; widths[a]=1;
        a=g.addArc(n2,n3); acolors[a]=0; widths[a]=1;
        a=g.addArc(n3,n5); acolors[a]=0; widths[a]=3;
        a=g.addArc(n5,n4); acolors[a]=0; widths[a]=1;
        a=g.addArc(n4,n1); acolors[a]=0; widths[a]=1;
        a=g.addArc(n2,n4); acolors[a]=1; widths[a]=2;
        a=g.addArc(n3,n4); acolors[a]=2; widths[a]=1;

        IdMap<ListDigraph,Node> id(g);

        // Create .eps files showing the digraph with different options
        cout << "Create 'graph_to_eps_demo_out_1_pure.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_1_pure.eps").
            coords(coords).
            title("Sample .eps figure").
            copyright("(C) 2003-2009 LEMON Project").
            run();

        cout << "Create 'graph_to_eps_demo_out_2.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_2.eps").
            coords(coords).
            title("Sample .eps figure").
            copyright("(C) 2003-2009 LEMON Project").
            absoluteNodeSizes().absoluteArcWidths().
            nodeScale(2).nodeSizes(sizes).
            nodeShapes(shapes).
            nodeColors(composeMap(palette,colors)).
            arcColors(composeMap(palette,acolors)).
            arcWidthScale(.4).arcWidths(widths).
            nodeTexts(id).nodeTextSize(3).
            run();

        cout << "Create 'graph_to_eps_demo_out_3_arr.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_3_arr.eps").
            title("Sample .eps figure (with arrowheads)").
            copyright("(C) 2003-2009 LEMON Project").
            absoluteNodeSizes().absoluteArcWidths().
            nodeColors(composeMap(palette,colors)).
            coords(coords).
            nodeScale(2).nodeSizes(sizes).
            nodeShapes(shapes).
            arcColors(composeMap(palette,acolors)).
            arcWidthScale(.4).arcWidths(widths).
            nodeTexts(id).nodeTextSize(3).
            drawArrows().arrowWidth(2).arrowLength(2).
            run();

        // Add more arcs to the digraph
        a=g.addArc(n1,n4); acolors[a]=2; widths[a]=1;
        a=g.addArc(n4,n1); acolors[a]=1; widths[a]=2;

        a=g.addArc(n1,n2); acolors[a]=1; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=2; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=3; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=4; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=5; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=6; widths[a]=1;
        a=g.addArc(n1,n2); acolors[a]=7; widths[a]=1;

        cout << "Create 'graph_to_eps_demo_out_4_par.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_4_par.eps").
            title("Sample .eps figure (parallel arcs)").
            copyright("(C) 2003-2009 LEMON Project").
            absoluteNodeSizes().absoluteArcWidths().
            nodeShapes(shapes).
            coords(coords).
            nodeScale(2).nodeSizes(sizes).
            nodeColors(composeMap(palette,colors)).
            arcColors(composeMap(palette,acolors)).
            arcWidthScale(.4).arcWidths(widths).
            nodeTexts(id).nodeTextSize(3).
            enableParallel().parArcDist(1.5).
            run();

        cout << "Create 'graph_to_eps_demo_out_5_par_arr.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_5_par_arr.eps").
            title("Sample .eps figure (parallel arcs and arrowheads)").
            copyright("(C) 2003-2009 LEMON Project").
            absoluteNodeSizes().absoluteArcWidths().
            nodeScale(2).nodeSizes(sizes).
            coords(coords).
            nodeShapes(shapes).
            nodeColors(composeMap(palette,colors)).
            arcColors(composeMap(palette,acolors)).
            arcWidthScale(.3).arcWidths(widths).
            nodeTexts(id).nodeTextSize(3).
            enableParallel().parArcDist(1).
            drawArrows().arrowWidth(1).arrowLength(1).
            run();

        cout << "Create 'graph_to_eps_demo_out_6_par_arr_a4.eps'" << endl;
        graphToEps(g,"/tmp/graph_to_eps_demo_out_6_par_arr_a4.eps").
            title("Sample .eps figure (fits to A4)").
            copyright("(C) 2003-2009 LEMON Project").
            scaleToA4().
            absoluteNodeSizes().absoluteArcWidths().
            nodeScale(2).nodeSizes(sizes).
            coords(coords).
            nodeShapes(shapes).
            nodeColors(composeMap(palette,colors)).
            arcColors(composeMap(palette,acolors)).
            arcWidthScale(.3).arcWidths(widths).
            nodeTexts(id).nodeTextSize(3).
            enableParallel().parArcDist(1).
            drawArrows().arrowWidth(1).arrowLength(1).
            run();

        // Create an .eps file showing the colors of a default Palette
        ListDigraph h;
        ListDigraph::NodeMap<int> hcolors(h);
        ListDigraph::NodeMap<Point> hcoords(h);

        int cols=int(std::sqrt(double(palette.size())));
        for(int i=0;i<int(paletteW.size());i++) {
        Node n=h.addNode();
            hcoords[n]=Point(1+i%cols,1+i/cols);
            hcolors[n]=i;
        }

        cout << "Create 'graph_to_eps_demo_out_7_colors.eps'" << endl;
        graphToEps(h,"/tmp/graph_to_eps_demo_out_7_colors.eps").
            scale(60).
            title("Sample .eps figure (Palette demo)").
            copyright("(C) 2003-2009 LEMON Project").
            coords(hcoords).
            absoluteNodeSizes().absoluteArcWidths().
            nodeScale(.45).
            distantColorNodeTexts().
            nodeTexts(hcolors).nodeTextSize(.6).
            nodeColors(composeMap(paletteW,hcolors)).
            run();

    }

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
                    if(targetLayer == CLIP_H_LEVEL_FINAL)
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

        if(!level)
            al_draw_filled_circle(x, y, TREEDRAWER_H_NODE_RADIUS + 10, circleColor);
        else
            al_draw_filled_circle(x, y, TREEDRAWER_H_NODE_RADIUS, circleColor);
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
