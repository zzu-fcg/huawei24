package com.huawei.codecraft;

import java.util.*;

public class Bfs {

    char[][] map;
    boolean[][] gds_map;

    public Bfs(char[][] map) {
        this.map = map;
    }

    public Bfs(char[][] map, boolean[][] gds_map) {
        this.map = map;
        this.gds_map = gds_map;
    }

    public Deque<int[]> bfsSearchGoodEarlyStop(int[] begin, int[] end) {
        //移动的四个方向
        int[][] dirs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        //储存未进行处理的点
        Queue<int[]> que = new LinkedList<int[]>();
        Deque<int[]> path = new LinkedList<int[]>();
        Map<String, int[]> father = new HashMap<String, int[]>();
        //记录是否访问过该位置
        boolean[][] visited = new boolean[map.length][map[0].length];
        //将起始点放入队列
        que.offer(begin);
        //标记为已访问
        visited[begin[0]][begin[1]] = true;
        //一直循环直到队列为空
        while (!que.isEmpty()) {
            //取出队列中最前端的点
            int[] current = que.poll();
            //寻路过程中提前找到货物或找到目标结点
            if (gds_map[current[0]][current[1]] || (current[0] == end[0] && current[1] == end[1])) {
                while (current != null) {
                    path.add(current);
                    current = father.get(Arrays.toString(current));
                }
                Collections.reverse((List) path);
                return path;
            }
            //四个方向循环
            for (int[] dir : dirs) {
                int x = current[0] + dir[0];
                int y = current[1] + dir[1];
                //判断是否可以走
                if (x < 0 || x > 199 || y < 0 || y > 199 || map[x][y] == '*' || map[x][y] == '#' || map[x][y] == '^' || visited[x][y]) {
                    continue;
                }
                //并将该点放入队列等待下次处理
                que.offer(new int[]{x, y});
                visited[x][y] = true;
                //记录该点的父节点
                father.put(Arrays.toString(new int[]{x, y}), new int[]{current[0], current[1]});
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回null
        return new LinkedList<>();
    }

    public Deque<int[]> bfsSearchGood(int[] begin, int[] end) {
        //移动的四个方向
        int[][] dirs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        //储存未进行处理的点
        Queue<int[]> que = new LinkedList<int[]>();
        Deque<int[]> path = new LinkedList<int[]>();
        Map<String, int[]> father = new HashMap<String, int[]>();
        //记录是否访问过该位置
        boolean[][] visited = new boolean[map.length][map[0].length];
        //将起始点放入队列
        que.offer(begin);
        //标记为已访问
        visited[begin[0]][begin[1]] = true;
        //一直循环直到队列为空
        while (!que.isEmpty()) {
            //取出队列中最前端的点
            int[] current = que.poll();
            //寻路过程中提前找到货物或找到目标结点
            if ((current[0] == end[0] && current[1] == end[1])) {
                while (current != null) {
                    path.add(current);
                    current = father.get(Arrays.toString(current));
                }
                Collections.reverse((List) path);
                return path;
            }
            //四个方向循环
            for (int[] dir : dirs) {
                int x = current[0] + dir[0];
                int y = current[1] + dir[1];
                //判断是否可以走
                if (x < 0 || x > 199 || y < 0 || y > 199 || map[x][y] == '*' || map[x][y] == '#' || map[x][y] == '^' || visited[x][y]) {
                    continue;
                }
                //并将该点放入队列等待下次处理
                que.offer(new int[]{x, y});
                visited[x][y] = true;
                //记录该点的父节点
                father.put(Arrays.toString(new int[]{x, y}), new int[]{current[0], current[1]});
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回null
        return new LinkedList<>();
    }

    public Deque<int[]> bfsSearchBerthEarlyStop(int[] begin, int[] end) {
        //移动的四个方向
        int[][] dirs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        //储存未进行处理的点
        Queue<int[]> que = new LinkedList<int[]>();
        Deque<int[]> path = new LinkedList<int[]>();
        Map<String, int[]> father = new HashMap<String, int[]>();
        //记录是否访问过该位置
        boolean[][] visited = new boolean[map.length][map[0].length];
        //将起始点放入队列
        que.offer(begin);
        //标记为已访问
        visited[begin[0]][begin[1]] = true;
        //一直循环直到队列为空
        while (!que.isEmpty()) {
            //取出队列中最前端的点
            int[] current = que.poll();
            //寻路过程中提前找到泊位或找到目标结点
            if (map[current[0]][current[1]] == 'B' || (current[0] == end[0] && current[1] == end[1])) {
                while (current != null) {
                    path.add(current);
                    current = father.get(Arrays.toString(current));
                }
                Collections.reverse((List) path);
                return path;
            }
            //四个方向循环
            for (int[] dir : dirs) {
                int x = current[0] + dir[0];
                int y = current[1] + dir[1];
                //判断是否可以走
                if (x < 0 || x > 199 || y < 0 || y > 199 || map[x][y] == '*' || map[x][y] == '#' || map[x][y] == '^' || visited[x][y]) {
                    continue;
                }
                //并将该点放入队列等待下次处理
                que.offer(new int[]{x, y});
                visited[x][y] = true;
                //记录该点的父节点
                father.put(Arrays.toString(new int[]{x, y}), new int[]{current[0], current[1]});
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回null
        return new LinkedList<>();
    }

    public Deque<int[]> bfsSearchBerth(int[] begin, int[] end) {
        //移动的四个方向
        int[][] dirs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        //储存未进行处理的点
        Queue<int[]> que = new LinkedList<int[]>();
        Deque<int[]> path = new LinkedList<int[]>();
        Map<String, int[]> father = new HashMap<String, int[]>();
        //记录是否访问过该位置
        boolean[][] visited = new boolean[map.length][map[0].length];
        //将起始点放入队列
        que.offer(begin);
        //标记为已访问
        visited[begin[0]][begin[1]] = true;
        //一直循环直到队列为空
        while (!que.isEmpty()) {
            //取出队列中最前端的点
            int[] current = que.poll();
            //寻路过程中提前找到货物或找到目标结点
            if ((current[0] == end[0] && current[1] == end[1])) {
                while (current != null) {
                    path.add(current);
                    current = father.get(Arrays.toString(current));
                }
                Collections.reverse((List) path);
                return path;
            }
            //四个方向循环
            for (int[] dir : dirs) {
                int x = current[0] + dir[0];
                int y = current[1] + dir[1];
                //判断是否可以走
                if (x < 0 || x > 199 || y < 0 || y > 199 || map[x][y] == '*' || map[x][y] == '#' || map[x][y] == '^' || visited[x][y]) {
                    continue;
                }
                //并将该点放入队列等待下次处理
                que.offer(new int[]{x, y});
                visited[x][y] = true;
                //记录该点的父节点
                father.put(Arrays.toString(new int[]{x, y}), new int[]{current[0], current[1]});
            }
        }
        //如果遍历完所有出现的结点都没有找到最终的结点，返回null
        return new LinkedList<>();
    }
}
