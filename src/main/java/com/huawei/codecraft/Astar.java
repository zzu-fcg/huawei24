package com.huawei.codecraft;

import java.util.*;

public class Astar {
    private static class Node implements Comparable<Node> {
        int x;  //x坐标
        int y;  //y坐标
        int F;  //F属性
        int G;  //G属性 节点n距离起点的代价
        int H;  //H属性 节点n距离终点的预计代价
        Node father;    //此结点的上一个结点

        //获取当前结点的坐标
        Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        //通过结点的坐标可以得到F， G， H三个属性
        //需要传入这个节点的上一个节点和最终的结点
        void init_node(Node father, Node end) {
            this.father = father;
            if (this.father != null) {
                this.G = father.G + 1;
            } else { //父节点为空代表它是第一个结点
                this.G = 0;
            }
            //计算通过现在的结点的位置和最终结点的位置计算H值
            this.H = Math.abs(this.x - end.x) + Math.abs(this.y - end.y);
            this.F = this.G + this.H;
        }

        // 用来进行和其他的Node类进行比较
        @Override
        public int compareTo(Node o) {
            return Integer.compare(this.F, o.F);
        }
    }

    char[][] map;
    boolean[][] gds_map;
    boolean[][] exist = new boolean[200][200];
    PriorityQueue<Node> Open = new PriorityQueue<Node>();
    //Close表用普通的数组
    ArrayList<Node> Close = new ArrayList<Node>();
    //Exist表用来存放已经出现过的结点。
    ArrayList<Node> Exist = new ArrayList<Node>();

    public Astar(char[][] map) {
        this.map = map;
    }

    public Astar(char[][] map, boolean[][] gds_map) {
        this.map = map;
        this.gds_map = gds_map;
    }

    public Deque<int[]> aStarSearchGoodEarlyStop(int[] sta, int[] en, StringBuilder msg) {
        //初始化结点
        Node start = new Node(sta[0], sta[1]);
        start.father = null;
        Node end = new Node(en[0], en[1]);
        //把第一个开始的结点加入到Open表中
        this.Open.add(start);
        //把出现过的结点加入到Exist表中
        //this.Exist.add(start);
        exist[sta[0]][sta[1]] = true;
        //主循环
        while (!Open.isEmpty()) {
            //取优先队列顶部元素并且把这个元素从Open表中删除
            Node current_node = Open.poll();
            //将这个结点加入到Close表中
            Close.add(current_node);
            //对当前结点进行扩展，得到一个四周结点的数组
            ArrayList<Node> neighbour_node = extend_current_node(current_node);
            //对这个结点遍历，看是否有目标结点出现
            //没有出现目标结点再看是否出现过
            for (Node node : neighbour_node) {
                //寻路过程中提前找到货物或找到目标结点
                if (gds_map[node.x][node.y] || (node.x == end.x && node.y == end.y)) { //寻路过程中提前找到货物
                    node.init_node(current_node, end);
                    return nodeQueue(node);
                }
                if (!is_exist(node)) {  //没出现过的结点加入到Open表中并且设置父节点
                    node.init_node(current_node, end);
                    Open.add(node);
                    //Exist.add(node);
                    exist[node.x][node.y] = true;
                }
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回empty
//        Deque<int[]> fake_path = new LinkedList<>();
//        fake_path.add(new int[]{-1, -1});
        return new LinkedList<>();
    }
    public Deque<int[]> aStarSearchGood(int[] sta, int[] en, StringBuilder msg) {
        //初始化结点
        Node start = new Node(sta[0], sta[1]);
        start.father = null;
        Node end = new Node(en[0], en[1]);
        //把第一个开始的结点加入到Open表中
        this.Open.add(start);
        //把出现过的结点加入到Exist表中
        //this.Exist.add(start);
        exist[sta[0]][sta[1]] = true;
        //主循环
        while (!Open.isEmpty()) {
            //取优先队列顶部元素并且把这个元素从Open表中删除
            Node current_node = Open.poll();
            //将这个结点加入到Close表中
            Close.add(current_node);
            //对当前结点进行扩展，得到一个四周结点的数组
            ArrayList<Node> neighbour_node = extend_current_node(current_node);
            //对这个结点遍历，看是否有目标结点出现
            //没有出现目标结点再看是否出现过
            for (Node node : neighbour_node) {
                //找到目标结点
                if (node.x == end.x && node.y == end.y) {
                    node.init_node(current_node, end);
                    return nodeQueue(node);
                }
                if (!is_exist(node)) {  //没出现过的结点加入到Open表中并且设置父节点
                    node.init_node(current_node, end);
                    Open.add(node);
                    //Exist.add(node);
                    exist[node.x][node.y] = true;
                }
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回empty
//        Deque<int[]> fake_path = new LinkedList<>();
//        fake_path.add(new int[]{-1, -1});
        return new LinkedList<>();
    }

    public Deque<int[]> aStarSearchBerth(int[] sta, int[] en) {
        //初始化结点
        Node start = new Node(sta[0], sta[1]);
        start.father = null;
        Node end = new Node(en[0], en[1]);
        //把第一个开始的结点加入到Open表中
        this.Open.add(start);
        //把出现过的结点加入到Exist表中
        exist[sta[0]][sta[1]] = true;
        //主循环
        while (!Open.isEmpty()) {
            //取优先队列顶部元素并且把这个元素从Open表中删除
            Node current_node = Open.poll();
            //将这个结点加入到Close表中
            Close.add(current_node);
            //对当前结点进行扩展，得到一个四周结点的数组
            ArrayList<Node> neighbour_node = extend_current_node(current_node);
            //对这个结点遍历，看是否有目标结点出现
            //没有出现目标结点再看是否出现过
            for (Node node : neighbour_node) {
//                if (map[node.x][node.y] == 'B') { //寻路过程中提前找到泊口
//                    node.init_node(current_node, end);
//                    return nodeQueue(node);
//                }
                if (node.x == end.x && node.y == end.y) {//找到目标结点就返回
                    node.init_node(current_node, end);
                    return nodeQueue(node);
                }
                if (!is_exist(node)) {  //没出现过的结点加入到Open表中并且设置父节点
                    node.init_node(current_node, end);
                    Open.add(node);
                    exist[node.x][node.y] = true;
                }
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回null
        return new LinkedList<>();
    }

    Deque<int[]> nodeQueue(Node node) {
        Deque<int[]> res = new LinkedList<>();
        while (node != null) {
            res.add(new int[]{node.x, node.y});
            node = node.father;
        }
        Collections.reverse((List) res);
        return res;
    }


    ArrayList<Node> extend_current_node(Node current_node) {
        int x = current_node.x;
        int y = current_node.y;
        ArrayList<Node> neighbour_node = new ArrayList<Node>();
        if (is_valid(x + 1, y)) {
            Node node = new Node(x + 1, y);
            neighbour_node.add(node);
        }
        if (is_valid(x - 1, y)) {
            Node node = new Node(x - 1, y);
            neighbour_node.add(node);
        }
        if (is_valid(x, y + 1)) {
            Node node = new Node(x, y + 1);
            neighbour_node.add(node);
        }
        if (is_valid(x, y - 1)) {
            Node node = new Node(x, y - 1);
            neighbour_node.add(node);
        }
        return neighbour_node;
    }

    boolean is_valid(int x, int y) {
        // 结点的位置不合法，返回
        if (x < 0 || x > 199 || y < 0 || y > 199 || map[x][y] == '*' || map[x][y] == '#' || map[x][y] == '^')
            return false;
//        for (Node node : Exist) {
//            //如果结点出现过，不合法
//            if (is_exist(new Node(x, y))) {
//                return false;
//            }
//        }
        return !is_exist(new Node(x, y));
    }

    boolean is_exist(Node node) {
        return exist[node.x][node.y];
    }
}