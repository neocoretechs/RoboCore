package com.neocoretechs.robocore;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Class to manage thread resources throughout the application. Singleton. Fixed thread pool.
 * Attached a run completion method that decrements a countdown latch until all threads up to executionLimit
 * have completed.
 * @author jg
 *
 */
public class SynchronizedFixedThreadPoolManager {
	int threadNum = 0;
    DaemonThreadFactory dtf ;//= new PoolThreadFactory();
    private static Map<String, ExecutorService> executor = new HashMap<String, ExecutorService>();// = Executors.newCachedThreadPool(dtf);
	public static SynchronizedFixedThreadPoolManager threadPoolManager = null;
	int totalThreads;
	private SynchronizedFixedThreadPoolManager() { }
	
	public static SynchronizedFixedThreadPoolManager getInstance(int maxThreads, int executionLimit) {
		if( threadPoolManager == null ) {
			threadPoolManager = new SynchronizedFixedThreadPoolManager();
			threadPoolManager.dtf = getInstance(maxThreads, executionLimit).new DaemonThreadFactory("SYSTEMSYNC");
			threadPoolManager.totalThreads = executionLimit;
			// set up pool for system processes
			ThreadPoolExecutor tpx = getInstance(maxThreads, executionLimit).new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit));
			tpx.prestartAllCoreThreads();
			executor.put("SYSTEMSYNC",tpx );		
		}
		return threadPoolManager;
	}
	/**
	 * Create an array of Executors that manage a cached thread pool for
	 * reading topics. One thread pool per topic to notify listeners of data ready
	 * @param threadGroupNames The topics for which thread groups are established
	 */
	public static void init(int maxThreads, int executionLimit, String[] threadGroupNames) {
		for(String tgn : threadGroupNames) {
			executor.put(tgn, getInstance(maxThreads, executionLimit).new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit)));
		}
	}
	
	public void waitForGroupToFinish(String group) throws InterruptedException {
		((ExtendedExecutor)executor.get(group)).getLatch().await();
		// now reset latch
		((ExtendedExecutor)executor.get(group)).latch = new CountDownLatch(totalThreads);
	}
	
	public void waitForGroupToFinish() throws InterruptedException {
		((ExtendedExecutor)executor.get("SYSTEMSYNC")).getLatch().await();
		// now reset latch
		((ExtendedExecutor)executor.get("SYSTEMSYNC")).latch = new CountDownLatch(totalThreads);
	}
	
	public BlockingQueue<Runnable> getQueue(String group) {
		return ((ThreadPoolExecutor)executor.get(group)).getQueue();
	}
	
	public BlockingQueue<Runnable> getQueue() {
		return ((ThreadPoolExecutor)executor.get("SYSTEMSYNC")).getQueue();
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
	
	public void spin(Runnable r) {
	    executor.get("SYSTEMSYNC").execute(r);
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
	
	class ExtendedExecutor extends ThreadPoolExecutor {
		public CountDownLatch latch;
		public ExtendedExecutor(int maxThreads, int totalThreads, BlockingQueue<Runnable> threadQueue) {
			super(maxThreads, totalThreads, Long.MAX_VALUE, TimeUnit.DAYS, threadQueue, dtf );
			latch = new CountDownLatch(totalThreads);
		}
		public CountDownLatch getLatch() { return latch; }
		
		@Override
		protected void afterExecute(Runnable r, Throwable t) {
			super.afterExecute(r, t);
		  // regardless of errors, etc, count down the latch
			latch.countDown();
		}
		
	}
}
