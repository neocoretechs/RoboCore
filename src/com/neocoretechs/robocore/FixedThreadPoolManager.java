package com.neocoretechs.robocore;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;

/**
 * Class to manage thread resources throughout the application. Singleton. Fixed thread pool.
 * @author jg
 *
 */
public class FixedThreadPoolManager {
	int threadNum = 0;
    DaemonThreadFactory dtf ;//= new PoolThreadFactory();
    private static Map<String, ExecutorService> executor = new HashMap<String, ExecutorService>();// = Executors.newCachedThreadPool(dtf);

	public static FixedThreadPoolManager threadPoolManager = null;
	private FixedThreadPoolManager() { }
	
	public static FixedThreadPoolManager getInstance(int maxThreads) {
		if( threadPoolManager == null ) {
			threadPoolManager = new FixedThreadPoolManager();
			// set up pool for system processes
			ThreadPoolExecutor tpx = (ThreadPoolExecutor) Executors.newFixedThreadPool(maxThreads, getInstance(maxThreads).new DaemonThreadFactory("SYSTEMFIX"));
			tpx.prestartAllCoreThreads();
			executor.put("SYSTEMFIX",tpx );		
		}
		return threadPoolManager;
	}
	/**
	 * Create an array of Executors that manage a cached thread pool for
	 * reading topics. One thread pool per topic to notify listeners of data ready
	 * @param threadGroupNames The topics for which thread groups are established
	 */
	public static void init(int maxThreads, String[] threadGroupNames) {
		for(String tgn : threadGroupNames) {
			executor.put(tgn, Executors.newFixedThreadPool(maxThreads, getInstance(maxThreads).new DaemonThreadFactory(tgn)));
		}
	}
	
	public BlockingQueue<Runnable> getQueue(String group) {
		return ((ThreadPoolExecutor)executor.get(group)).getQueue();
	}
	
	public BlockingQueue<Runnable> getQueue() {
		return ((ThreadPoolExecutor)executor.get("SYSTEMFIX")).getQueue();
	}
	
	public void waitGroup(String group) {
		try {
			ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait();
			}
		} catch (InterruptedException e) {
		}
	}
	
	public void waitGroup(String group, long millis) {
		try {
			ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait(millis);
			}
		} catch (InterruptedException e) {
		}
	}
	
	public void notifyGroup(String group) {
			ExecutorService w = executor.get(group);
			synchronized(w) {
				w.notifyAll();
			}
	}
	
	public void spin(Runnable r, ThreadGroup group) {
	    executor.get(group.getName()).execute(r);
	}
	
	public void spin(Runnable r, String group) {
	    executor.get(group).execute(r);
	}
	
	public void shutdown() {
		Collection<ExecutorService> ex = executor.values();
		for(ExecutorService e : ex) {
			List<Runnable> spun = e.shutdownNow();
			for(Runnable rs : spun) {
				System.out.println("Marked for Termination:"+rs.toString()+" "+e.toString());
			}
		}
	}
	
	class DaemonThreadFactory implements ThreadFactory {
		ThreadGroup threadGroup;
	
		public DaemonThreadFactory(String threadGroupName) {
			threadGroup = new ThreadGroup(threadGroupName);
		}	
		public ThreadGroup getThreadGroup() { return threadGroup; }		
	    public Thread newThread(Runnable r) {
	        Thread thread = new Thread(threadGroup, r, threadGroup.getName()+(++threadNum));
	        //thread.setDaemon(true);
	        return thread;
	    }
	}
}
