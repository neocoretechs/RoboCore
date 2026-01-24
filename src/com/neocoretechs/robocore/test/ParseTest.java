package com.neocoretechs.robocore.test;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Stack;

import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;

public class ParseTest {
	static Stack<String> stack = new Stack<String>();
	static ArrayList<String> visited = new ArrayList<String>();
	
	private static String parseFile(File f, String xPath) throws Exception {
		Document doc = Jsoup.parse(f, "UTF-8", f.toURI().toString());
		Elements results = null;
		results = doc.selectXpath(xPath);
		StringBuilder sb = new StringBuilder();
		for (Element e : results) {
		    String t = e.text().trim();
		    if (!t.isEmpty()) {
		        sb.append(t).append("\n");
		    }
		}
		return sb.toString();
	}
	public static void parseLinks(File f) throws IOException {
		Document doc = Jsoup.parse(f, "UTF-8", f.toURI().toString());
		Elements links = doc.select("a[href]");
		for (Element link : links) {
		    String abs = link.attr("abs:href");
		    if (abs == null || abs.isEmpty()) continue;
		    // Only follow HTML pages
		    if (abs.contains("?") || !abs.endsWith(".html")) continue;
		    // Avoid reprocessing
		    if(!visited.contains(abs)) {
		    	visited.add(abs);
		        stack.push(abs);
		    }
		}
	}
	public static void extractContent() throws Exception {
		while (!stack.isEmpty()) {
		    String file = stack.pop();
		    System.out.println(">>> parsing file from link: " + file);
		    String fileTrim;
		    if (file.startsWith("file:/"))
		        fileTrim = file.substring(6);
		    else
		        fileTrim = file;

		    File f = new File(fileTrim);

		    // Extract real content
		    String desc = parseFile(f, "//div[@class='description']//div[@class='block']");
		    String methods = parseFile(f, "//table[contains(@class,'memberSummary')]//td[@class='colLast']");
		    String details = parseFile(f, "//div[@class='details']//div[contains(@class,'block')]");

		    if(desc != null && desc.trim().length() > 0)
		    	System.out.println("DESCRIPTION:\n" + desc);
		    if(methods != null && methods.trim().length() > 0)
		    	System.out.println("METHOD SUMMARY:\n" + methods);
		    if(details != null && details.trim().length() > 0)
		    	System.out.println("DETAILS:\n" + details);

		    // Continue traversal
		    parseLinks(f);
		}
	}
	public static void extractStructure() throws Exception {
		while (!stack.isEmpty()) {
		    String file = stack.pop();
		    String fileTrim;
			System.out.println(">>>parsing file from link:"+file);
			if(file.startsWith("file:/"))
				fileTrim = file.substring(6);
			else
				fileTrim = file;
			File f = new File(fileTrim);
		    String s = ParseTest.parseFile(f,"//a");
			if(s != null)
				System.out.println(s);
			else
				System.out.println("no text for "+f);
		}
	}
	public static void main(String[] args) throws Exception {
		String file = args[0];
		String fileTrim;
		if(file.startsWith("file://"))
			fileTrim = file.substring(7);
		else
			fileTrim = file;
		File f = new File(fileTrim);
		System.out.println(">>>parsing file:"+fileTrim);
		String s = ParseTest.parseFile(f,"//a");
		if(s != null)
			System.out.println(s);
		else
			System.out.println("no text for "+f);
		parseLinks(f);
		extractStructure();
		visited.clear();
		parseLinks(f);
		extractContent();
	}
}
